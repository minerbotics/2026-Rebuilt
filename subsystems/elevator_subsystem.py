import commands2
import wpilib
import phoenix6

from constants import ElevatorConstants

class ElevatorSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.elevator = phoenix6.hardware.TalonFXS(ElevatorConstants._elevator_id)
        motorArrangement = phoenix6.signals.MotorArrangementValue.NEO_JST
        self.arrangementConfig = phoenix6.configs.CommutationConfigs().with_motor_arrangement(motorArrangement)
        elevator_configurator = self.elevator.configurator
        elevator_configurator.apply(self.arrangementConfig)
    
    def up(self) -> None:
        self.elevator.set(0.1)
    
    def down(self) -> None:
        self.elevator.set(-0.1)
    
    def stop(self) -> None:
        self.elevator.set(0)
