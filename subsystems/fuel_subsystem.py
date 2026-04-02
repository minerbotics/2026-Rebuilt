import commands2
import wpilib
import phoenix6
import rev

from constants import FuelConstants

class FuelSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intakeRoller = phoenix6.hardware.TalonFXS(FuelConstants._intake_id)
        self.feederRoller = phoenix6.hardware.TalonFXS(FuelConstants._feeder_id)
        motorArrangement = phoenix6.signals.MotorArrangementValue.NEO_JST
        self.arrangementConfig = phoenix6.configs.CommutationConfigs().with_motor_arrangement(motorArrangement)
        launcher_configurator = self.intakeRoller.configurator
        feeder_configurator = self.feederRoller.configurator
        launcher_configurator.apply(self.arrangementConfig)
        feeder_configurator.apply(self.arrangementConfig)

        self.leftLauncher = rev.SparkFlex(FuelConstants._left_launcher_id, rev.SparkFlex.MotorType.kBrushless)
        self.rightLauncher = rev.SparkFlex(FuelConstants._right_launcher_id, rev.SparkFlex.MotorType.kBrushless)
        
        #follower_config = rev.SparkBaseConfig()
        #follower_config.follow(self.leftLauncher, True)
        #self.rightLauncher.configure(
        #    follower_config,
        #    rev.ResetMode.kResetSafeParameters,
        #    rev.PersistMode.kPersistParameters
        #    )


    def intake(self) -> None:
        self.feederRoller.set(-1 * FuelConstants._feeder_intake_speed)
        self.intakeRoller.set(-1 * FuelConstants._intake_intake_speed)

    def eject(self) -> None:
        self.feederRoller.set(FuelConstants._feeder_intake_speed)
        self.intakeRoller.set(FuelConstants._intake_intake_speed)

    def launch(self) -> None:
        self.feederRoller.set(FuelConstants._feeder_launch_speed)
        self.intakeRoller.set(-1 * FuelConstants._intake_launch_speed)
        self.leftLauncher.set(-1 * FuelConstants._launcher_launch_speed)
        self.rightLauncher.set(1 * FuelConstants._launcher_launch_speed)

    def spinUp(self) -> None:
        self.feederRoller.set(FuelConstants._feeder_spinup_speed)
        self.intakeRoller.set(-1 * FuelConstants._intake_launch_speed)
        self.leftLauncher.set(-1 * FuelConstants._launcher_launch_speed)
        self.rightLauncher.set(1 * FuelConstants._launcher_launch_speed)

    def stop(self) -> None:
        self.feederRoller.set(0)
        self.intakeRoller.set(0)
        self.leftLauncher.set(0)
        self.rightLauncher.set(0)
    
    def spinUpCommand(self) -> commands2.Command:
        return self.run(self.spinUp)
    
    def launchCommand(self) -> commands2.Command:
        return self.run(self.launch)