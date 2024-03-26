import commands2

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import Shooter

from commands.rotateIntake import RotateIntake
from commands.FlyWheelSpeed import FlyWheelSpeed
from commands.setIntakeSpeed import SetIntakeSpeed
from commands.ResetYaw import ResetYaw
from commands.zeroPose import zeroPose

from constants import IntakeConstants


class Shoot(commands2.SequentialCommandGroup):
    def __init__(
        self,
        _drive: DriveSubsystem,
        _intake: IntakeSubsystem,
        _shooter: Shooter,
    ):
        """
        This just shoots the note,
        useful for being with teams
        with super autos
        """
        super().__init__(
            # Resets Yaw relative to the robot's starting position
            ResetYaw(_drive),
            zeroPose(_drive),
            # ============ #
            # SPEAKER SHOT #
            # ============ #
            RotateIntake(
                IntakeConstants.BlowPos, _intake
            ),  # Put intake fully inside (if it wasn't already)
            FlyWheelSpeed(1.0, _shooter, False),  # Power up the flywheels (?)
            SetIntakeSpeed(-0.6, _intake),  # Load magazine? (but without ending)
            commands2.WaitCommand(1),
            FlyWheelSpeed(0.0, _shooter),  # Stop flywheel
            SetIntakeSpeed(0, _intake),  # Stop intake)
        )


NAME = "Shoot"
load = lambda bot: Shoot(bot.robotDrive, bot.intake, bot.shooter)
