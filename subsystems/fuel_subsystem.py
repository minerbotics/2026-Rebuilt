import commands2
import wpilib
import phoenix6

from constants import FuelConstants

class FuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intakeLauncherRoller = phoenix6.hardware.TalonFX(FuelConstants._launcher_id)
        self.feederRoller = phoenix6.hardware.TalonFX(FuelConstants._feeder_id)
        feederConfig = phoenix6.configs.TalonFXConfiguration()

    def intake(self) -> None:
        self.feederRoller.setVoltage(FuelConstants._feeder_voltage)
        self.intakeLauncherRoller.setVoltage(FuelConstants._launcher_voltage)

    def stop(self) -> None:
        self.feederRoller.set(0)
        self.intakeLauncherRoller.set(0)