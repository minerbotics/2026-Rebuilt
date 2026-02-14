import commands2
import wpilib
import phoenix6

from constants import FualConstants


class FuelSubsystem(commands2.subsystem):
    def __init__(self) -> None:
        super().__init__()
