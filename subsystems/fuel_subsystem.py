import commands2
import wpilib
import phoenix6
import rev

from constants import FuelConstants


class FuelSubsystem(commands2.Subsystem):

    def __init__(self) -> None:
        super().__init__()

        # Phoenix & feeder motors (unchanged)
        self._intake_roller = phoenix6.hardware.TalonFXS(FuelConstants._intake_id)
        self._feeder_roller = phoenix6.hardware.TalonFXS(FuelConstants._feeder_id)
        motor_arrangement = phoenix6.signals.MotorArrangementValue.NEO_JST
        arrangement_config = phoenix6.configs.CommutationConfigs().with_motor_arrangement(motor_arrangement)
        launcher_configurator = self._intake_roller.configurator
        feeder_configurator = self._feeder_roller.configurator
        launcher_configurator.apply(arrangement_config)
        feeder_configurator.apply(arrangement_config)

        # Launch motors (NEO Vortex / SparkFlex)
        self._left_launcher = rev.SparkFlex(FuelConstants._left_launcher_id, rev.SparkFlex.MotorType.kBrushless)
        self._right_launcher = rev.SparkFlex(FuelConstants._right_launcher_id, rev.SparkFlex.MotorType.kBrushless)

        # Closed-loop target and tolerance
        self._target_rpm = 0.0
        self._rpm_tolerance = getattr(FuelConstants, "_launcher_rpm_tolerance", 150)

        # Gains and feed-forward (defaults - tune on robot)
        self._kP = getattr(FuelConstants, "_launcher_kP", 0.0004)
        self._kI = getattr(FuelConstants, "_launcher_kI", 0.0)
        self._kD = getattr(FuelConstants, "_launcher_kD", 0.0)
        self._ff = getattr(FuelConstants, "_launcher_ff", 0.0)

        # Configure launch motors
        left_launch_config = rev.SparkFlexConfig()
        left_launch_config.setIdleMode(
            left_launch_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        )
        left_launch_config.smartCurrentLimit(40, freeLimit=6700)
        left_launch_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder).pid(
            self._kP, self._kI, self._kD
        )
        self._left_launcher.configure(
            left_launch_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        right_launch_config = rev.SparkFlexConfig()
        right_launch_config.setIdleMode(
            right_launch_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        )
        right_launch_config.smartCurrentLimit(40, freeLimit=6700)
        right_launch_config.follow(self._left_launcher, True)

        self._right_launcher.configure(
            right_launch_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        
        self._left_controller = self._left_launcher.getClosedLoopController()
        self._right_controller = self._right_launcher.getClosedLoopController()
        self._left_encoder = self._left_launcher.getEncoder()
        self._right_encoder = self._right_launcher.getEncoder()
        

    # Intake / eject functions (unchanged semantics)
    def intake(self) -> None:
        self._feeder_roller.set(-1 * FuelConstants._feeder_intake_speed)
        self._intake_roller.set(-1 * FuelConstants._intake_intake_speed)

    def eject(self) -> None:
        self._feeder_roller.set(FuelConstants._feeder_intake_speed)
        self._intake_roller.set(FuelConstants._intake_intake_speed)

    # High-level set launcher RPM (uses motor-controller closed-loop when available)
    def setLauncherRPM(self, rpm: float) -> None:
        """Set desired launcher RPM. Uses SparkClosedLoopController or motor-side PID if available."""
        self._target_rpm = float(rpm)

        if rpm == 0.0:
            # Stop immediately
            self._target_rpm = 0.0
            self._left_launcher.set(0)
            self._right_launcher.set(0)
            return

        # If closed-loop controller object exists (SparkClosedLoopController)
        if self._left_controller:
            try:
                # Keep sign convention: left negative, right positive
                if hasattr(self._left_controller, "setSetpoint"):
                    self._left_controller.setSetpoint(-self._target_rpm, rev.SparkFlex.ControlType.kVelocity)
                return
            except Exception:
                # fall through to other methods
                pass

        # Fallback open-loop guess
        open_guess = getattr(FuelConstants, "_launcher_launch_speed", 0.8)
        self._left_launcher.set(-open_guess)

    def stopLauncher(self) -> None:
        self._target_rpm = 0.0
        self._left_launcher.set(0)

    def isLauncherReady(self) -> bool:
        """Return True when both launchers are within rpm tolerance of target."""
        if self._target_rpm <= 0:
            return False
        if self._left_encoder is None or self._right_encoder is None:
            return False
        try:
            left_v = float(self._left_encoder.getVelocity())
            right_v = float(self._right_encoder.getVelocity())
        except Exception:
            return False

        # Compare absolute rpm values (encoders may give signed velocities)
        return (abs(abs(left_v) - self._target_rpm) <= self._rpm_tolerance and
                abs(abs(right_v) - self._target_rpm) <= self._rpm_tolerance)

    def launch(self) -> None:
        """
        High-level launch: set launcher RPM and run feeder/intake.
        Feeders are gated until launcher is at speed to avoid large droop under compression.
        """
        # Request the target rpm
        self.setLauncherRPM(FuelConstants._launcher_target_rpm)

        if getattr(FuelConstants, "_launcher_feed_when_ready", True):
            if self.isLauncherReady():
                self._feeder_roller.set(FuelConstants._feeder_launch_speed)
                self._intake_roller.set(-1 * FuelConstants._intake_launch_speed)
            else:
                self._feeder_roller.set(0)
                self._intake_roller.set(0)
        else:
            self._feeder_roller.set(FuelConstants._feeder_launch_speed)
            self._intake_roller.set(-1 * FuelConstants._intake_launch_speed)

    def spinUp(self) -> None:
        """Spin up launcher (don't feed)"""
        self.setLauncherRPM(FuelConstants._launcher_target_rpm)
        self._feeder_roller.set(0)
        self._intake_roller.set(0)

    def stop(self) -> None:
        self._feeder_roller.set(0)
        self._intake_roller.set(0)
        self._left_launcher.set(0)
        self._right_launcher.set(0)
        self._target_rpm = 0.0

    def spinUpCommand(self) -> commands2.Command:
        return self.run(self.spinUp)

    def launchCommand(self) -> commands2.Command:
        return self.run(self.launch)

    def periodic(self) -> None:
        """
        Light monitor. Closed-loop runs on controller hardware at high rate; here we optionally re-issue
        setReference if both sides drop very low (conservative fallback).
        """
        try:
            left_v = float(self._left_encoder.getVelocity())
            right_v = float(self._right_encoder.getVelocity())
        except Exception:
            return

        # Conservative corrective re-issue of reference if both sides fall far below target
        if self._target_rpm > 0 and (abs(left_v) < 0.7 * self._target_rpm and abs(right_v) < 0.7 * self._target_rpm):
            try:
                # Try controller objects first
                if self._left_controller and hasattr(self._left_controller, "setSetpoint"):
                    self._left_controller.setSetpoint(-self._target_rpm, rev.SparkFlex.ControlType.kVelocity)
                if self._right_controller and hasattr(self._right_controller, "setSetpoint"):
                    self._right_controller.setSetpoint(self._target_rpm, rev.SparkFlex.ControlType.kVelocity)
            except Exception:
                pass