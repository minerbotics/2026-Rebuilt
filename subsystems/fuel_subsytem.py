import commands2
import wpilib
import phoenix6

from constants import FualConstants


class FuelSubsystem(commands2.subsystem):
    def __init__(self) -> None:
        super().__init__()
#placeholder and voltage is subject to change
        self.intakeLauncherRoller = phoenix6.hardware.TalonFX(
            FuelConstants._launcher_id,
            phoenix6.Placeholder.Placeholder.Placeholder,
        )
        self.feederRoller = phoenix6.hardware.TalonFX(
            FuelConstants._feeder_id, phoenix6.Placeholder.Placeholder.Placeholder
        )

        wpilib.SmartDashboard.putNumber(
            "Intaking feeder roller value", FuelConstants._intaking_feeder_voltage
        )
        wpilib.SmartDashboard.putNumber(
            "Intaking intake roller value", FuelConstants._intacking_intake_voltage
        )
        wpilib.SmartDashboard.putNumber(
            "Launching feeder roller value", FuelConstants._launching_feeder_voltage
        )
        wpilib.SmartDashboard.putNumber(
            "Launching launcher roller value", FuelConstants._launching_launcher_voltage
        )
        
