import commands2
import wpilib
import phoenix6

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

    def intake(self) -> None:
        self.feederRoller.set(-0.5)
        self.launcherRoller.set(-0.3)

    def eject(self) -> None:
        self.feederRoller.set(0.5)
        self.launcherRoller.set(0.3)

    def launch(self) -> None:
        self.feederRoller.set(-0.3)
        self.launcherRoller.set(-0.7)

    def spinUp(self) -> None:
        self.feederRoller.set(-0.25)
        self.launcherRoller.set(-0.7)

    def stop(self) -> None:
        self.feederRoller.set(0)
        self.launcherRoller.set(0)