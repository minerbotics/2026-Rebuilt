import commands2
import wpilib
import phoenix6
import math
from typing import Optional

from wpimath.controller import PIDController, SimpleMotorFeedforward
from wpilib import SmartDashboard

from constants import FuelConstants

class FuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.launcherRoller = phoenix6.hardware.TalonFXS(FuelConstants._launcher_id)
        self.feederRoller = phoenix6.hardware.TalonFXS(FuelConstants._feeder_id)
        motorArrangement = phoenix6.signals.MotorArrangementValue.NEO_JST
        self.arrangementConfig = phoenix6.configs.CommutationConfigs().with_motor_arrangement(motorArrangement)
        launcher_configurator = self.launcherRoller.configurator
        feeder_configurator = self.feederRoller.configurator
        launcher_configurator.apply(self.arrangementConfig)
        feeder_configurator.apply(self.arrangementConfig)

        # Velocity control state
        # Target RPM for launcher; defaults to configured constant
        self._target_rpm: float = float(getattr(FuelConstants, "_launcher_launch_speed", 3000.0))

        # Try to use onboard closed-loop on the controller first; otherwise fall back to rio PID
        self._onboard_velocity_available = False

        # Robot-side PID + Feedforward (fallback)
        self._launch_pid = PIDController(
            getattr(FuelConstants, "_launcher_kP", 0.0015),
            getattr(FuelConstants, "_launcher_kI", 0.0),
            getattr(FuelConstants, "_launcher_kD", 0.0002),
        )
        self._launch_ff = SimpleMotorFeedforward(
            getattr(FuelConstants, "_launcher_ff_kS", 0.0),
            getattr(FuelConstants, "_launcher_ff_kV", 0.00015),
            getattr(FuelConstants, "_launcher_ff_kA", 0.0),
        )

        # Vision subsystem (optional) can be injected via set_vision
        self.vision = None

        # Telemetry
        SmartDashboard.putNumber("Launcher/target_rpm", self._target_rpm)
        SmartDashboard.putNumber("Launcher/current_rpm", 0.0)

    def intake(self) -> None:
        self.feederRoller.set(-1 * FuelConstants._feeder_intake_speed)
        self.launcherRoller.set(-1 * FuelConstants._launcher_intake_speed)

    def eject(self) -> None:
        self.feederRoller.set(FuelConstants._feeder_intake_speed)
        self.launcherRoller.set(FuelConstants._launcher_intake_speed)

    def launch(self) -> None:
        # Set feeder open-loop and command launcher to desired velocity
        self.feederRoller.set(FuelConstants._feeder_launch_speed)
        # compute target rpm from vision if available; otherwise use constant
        self._target_rpm = float(getattr(FuelConstants, "_launcher_launch_speed", self._target_rpm))
        if self.vision is not None:
            computed = self._compute_target_rpm_from_vision()
            if computed is not None:
                self._target_rpm = computed

        # Command velocity using onboard first, fallback to rio PID
        self._command_velocity(self._target_rpm)

    def spinUp(self) -> None:
        self.feederRoller.set(FuelConstants._feeder_spinup_speed)
        # spinup uses a lower target rpm (use launch speed by default)
        self._target_rpm = float(getattr(FuelConstants, "_launcher_spinup_rpm", getattr(FuelConstants, "_launcher_launch_speed", 3000.0)))
        self._command_velocity(self._target_rpm)

    def stop(self) -> None:
        self.feederRoller.set(0)
        # Stop closed-loop attempts and motor
        self._target_rpm = 0.0
        try:
            self.launcherRoller.set(0)
        except Exception:
            pass
    
    def spinUpCommand(self) -> commands2.Command:
        return self.run(self.spinUp)
    
    def launchCommand(self) -> commands2.Command:
        return self.run(self.launch)

    def set_vision(self, vision_subsystem) -> None:
        """Attach a vision subsystem instance for dynamic RPM calculation."""
        self.vision = vision_subsystem

    def _compute_target_rpm_from_vision(self) -> Optional[float]:
        """Compute a target RPM based on vision result. Returns None if no update.

        This uses a placeholder mapping: if the vision result exposes a targetPitch (degrees)
        we estimate distance and map distance to RPM linearly between min/max RPM constants.
        Replace with an empirically tuned mapping for your robot.
        """
        try:
            result = self.vision.get_latest_result()
            if result is None:
                return None
            # Best target
            target = None
            if hasattr(result, "getBestTarget"):
                target = result.getBestTarget()
            elif hasattr(result, "get_best_target"):
                target = result.get_best_target()
            elif hasattr(result, "targets") and len(getattr(result, "targets", [])) > 0:
                target = result.targets[0]

            if target is None:
                return None

            # Try to read pitch (degrees)
            pitch = None
            if hasattr(target, "getPitch"):
                pitch = float(target.getPitch())
            elif hasattr(target, "pitch"):
                pitch = float(getattr(target, "pitch"))

            if pitch is None:
                return None

            # Example distance estimate using camera pitch and assumed target height constant
            # Use CAMERA_HEIGHT and TARGET_HEIGHT in constants if available
            cam_height_m = getattr(self.vision, "camera_height_m", None)
            cam_pitch_rad = getattr(self.vision, "camera_pitch_rad", None)
            target_height_m = getattr(FuelConstants, "_target_height_m", None)
            if cam_height_m is None or cam_pitch_rad is None or target_height_m is None:
                # can't compute distance reliably
                return None

            # Convert pitch from degrees to radians (photonlib returns degrees)
            pitch_rad = math.radians(pitch)
            # distance = (target_height - cam_height) / tan(cam_pitch + pitch)
            denom = math.tan(cam_pitch_rad + pitch_rad)
            if denom == 0:
                return None
            distance_m = (target_height_m - cam_height_m) / denom

            # Map distance to RPM linearly between constants (placeholders)
            min_rpm = float(getattr(FuelConstants, "_launcher_min_rpm", 1000.0))
            max_rpm = float(getattr(FuelConstants, "_launcher_max_rpm", 6000.0))
            # clamp distance range
            min_dist = float(getattr(FuelConstants, "_target_min_dist_m", 0.5))
            max_dist = float(getattr(FuelConstants, "_target_max_dist_m", 6.0))
            # inverse mapping: closer -> lower RPM, farther -> higher RPM
            t = (distance_m - min_dist) / (max_dist - min_dist)
            t = max(0.0, min(1.0, t))
            rpm = min_rpm + t * (max_rpm - min_rpm)
            return rpm
        except Exception:
            return None

    def get_launcher_velocity_rpm(self) -> Optional[float]:
        """Read launcher velocity from phoenix6 observed_state or controller; return RPM if available."""
        try:
            obs = getattr(self.launcherRoller, "observed_state", None)
            if obs is not None:
                v = getattr(obs, "rotor_velocity", None) or getattr(obs, "velocity", None)
                if v is not None:
                    # If large (>1000) assume it's already RPM, otherwise assume rad/s
                    if abs(v) > 1000:
                        return float(v)
                    return float(v) * 60.0 / (2.0 * math.pi)
        except Exception:
            pass

        # try controller getter
        try:
            if hasattr(self.launcherRoller, "getSelectedSensorVelocity"):
                units = self.launcherRoller.getSelectedSensorVelocity()
                # Unknown units; return raw for now
                return float(units)
        except Exception:
            pass

        return None

    def _command_velocity(self, target_rpm: float) -> None:
        """Attempt to command the launcher to target_rpm. Use onboard velocity control if available,
        otherwise use robot-side PID+feedforward to set percent output.
        """
        # Try onboard controller API first
        try:
            if hasattr(self.launcherRoller, "set_velocity"):
                # some phoenix6 bindings expect RPM or rad/s; assume RPM here
                self.launcherRoller.set_velocity(target_rpm)
                self._onboard_velocity_available = True
                SmartDashboard.putNumber("Launcher/target_rpm", target_rpm)
                return
            # Try generic set(controlMode, value) style
            if hasattr(self.launcherRoller, "set"):
                try:
                    # Some bindings may accept a tuple or ControlMode - try common pattern
                    from phoenix.motorcontrol import ControlMode  # type: ignore
                    self.launcherRoller.set(ControlMode.Velocity, target_rpm)
                    self._onboard_velocity_available = True
                    SmartDashboard.putNumber("Launcher/target_rpm", target_rpm)
                    return
                except Exception:
                    pass
        except Exception:
            pass

        # Fallback: robot-side PID + FF
        current_rpm = self.get_launcher_velocity_rpm()
        if current_rpm is None:
            # Can't close loop; approximate percent using ratio to max_rpm
            max_rpm = float(getattr(FuelConstants, "_launcher_max_rpm", 6000.0))
            pct = max(0.0, min(1.0, target_rpm / max_rpm))
            try:
                self.launcherRoller.set(-1 * pct)
            except Exception:
                try:
                    self.launcherRoller.setVoltage(-pct * 12.0)
                except Exception:
                    pass
            SmartDashboard.putNumber("Launcher/target_rpm", target_rpm)
            return

        pid_output = self._launch_pid.calculate(current_rpm, target_rpm)
        ff_volts = self._launch_ff.calculate(target_rpm / 60.0)
        bus_voltage = 12.0
        voltage_set = max(min(ff_volts + pid_output * bus_voltage, bus_voltage), -bus_voltage)

        try:
            # if the binding supports setVoltage
            if hasattr(self.launcherRoller, "setVoltage"):
                self.launcherRoller.setVoltage(-voltage_set)
            else:
                # fallback to percent
                self.launcherRoller.set(-voltage_set / bus_voltage)
        except Exception:
            try:
                self.launcherRoller.set(-voltage_set / bus_voltage)
            except Exception:
                pass

        SmartDashboard.putNumber("Launcher/current_rpm", current_rpm)
        SmartDashboard.putNumber("Launcher/target_rpm", target_rpm)