import commands2
import wpilib
import phoenix6

from constants import FuelConstants

class FuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intakeLauncherRoller = phoenix6.hardware.TalonFX(FuelConstants._launcher_id)
        self.feederRoller = phoenix6.hardware.TalonFX(FuelConstants._feeder_id)
        motorConfig = phoenix6.configs.TalonFXConfiguration()
        motorConfig.Commutation.MotorArrangement = phoenix6.signals.MotorArrangementValue.Minion_JST
        self.intakeLauncherRoller.getConfiguration().apply(motorConfig)
        self.feederRoller.getConfiguration().apply(motorConfig)

    def intake(self) -> None:
        self.feederRoller.set(0.1)
        self.intakeLauncherRoller.set(0.1)

    def stop(self) -> None:
        self.feederRoller.set(0)
        self.intakeLauncherRoller.set(0)