import commands2

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import Shooter

from commands.FlyWheelSpeed import FlyWheelSpeed
from commands.setIntakeSpeed import SetIntakeSpeed

from autonomous.flywithwires import FlyWithWires


class NothDrive(commands2.SequentialCommandGroup):
    def __init__(
        self,
        driveSubsystem: DriveSubsystem,
        shooterSubsytem: Shooter,
        intakeSubsystem: IntakeSubsystem,
    ) -> None:
        """
        This shoots a note and drives backwards
        it assumes you are starting on the mothing side
        this wants the note to start in the intake (folded up)
        with the limit switch pressed
        """

        super().__init__(
            FlyWheelSpeed(1, shooterSubsytem, False),  # Power up the flywheels
            SetIntakeSpeed(-0.6, intakeSubsystem),  # Load magazine
            commands2.WaitCommand(
                3
            ),  # this needs to be here because it won't hold speed otherwise
            FlyWheelSpeed(0.0, shooterSubsytem),  # Stop flywheel
            SetIntakeSpeed(0, intakeSubsystem).withTimeout(3),  # Stop intake
            FlyWithWires(driveSubsystem, 1, 0, 0, 1),  # driving
            FlyWithWires(driveSubsystem, 0, 0, 0.2, 1),  # turning
            FlyWithWires(driveSubsystem, 1, 0, 0, 1),  # driving
        )
