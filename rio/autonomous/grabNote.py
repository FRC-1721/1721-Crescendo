import commands2

from constants import SuperStrucConstants
from subsystems.limelight import limeLightCommands
from subsystems.shooter import Shooter
from subsystems.intake import IntakeSubsystem
from subsystems.drivesubsystem import DriveSubsystem

from commands.rotateIntake import RotateIntake
from commands.FlyWheelSpeed import FlyWheelSpeed
from commands.setIntakeSpeed import SetIntakeSpeed
from commands.ResetYaw import ResetYaw
from commands.NavToObj import sendToObject
from commands.shooterROT import ShooterROT
from commands.intakeUntilNote import intakeUntilNote
from commands.poseReset import PoseReset
from commands.zeroPose import zeroPose

from constants import IntakeConstants


class GrabNote(commands2.SequentialCommandGroup):
    def __init__(
        self,
        _limelight: limeLightCommands,
        _shooter: Shooter,
        _intake: IntakeSubsystem,
        _drive: DriveSubsystem,
    ) -> None:
        """
        Runs around and shoots as many speaker shots as possible
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
            print("intake rotatered"),
            FlyWheelSpeed(1.0, _shooter, False),  # Power up the flywheels (?)
            print("wheels r fly"),
            SetIntakeSpeed(-0.6, _intake),  # Load magazine? (but without ending)
            commands2.WaitCommand(1),
            FlyWheelSpeed(0.0, _shooter),  # Stop flywheel
            SetIntakeSpeed(0, _intake),  # Stop intake
            # ================ #
            # COLLECT NEW NOTE #
            # ================ #
            sendToObject(_drive, _limelight),
            ShooterROT(SuperStrucConstants.LoadPos, _shooter),
            FlyWheelSpeed(0, _shooter),  # Stop shooter (if its running)
            RotateIntake(
                IntakeConstants.SuckPos, _intake
            ),  # Put intake down (with a lil extra squeeze)
            intakeUntilNote(0.5, _intake),  # Intake till note
            RotateIntake(IntakeConstants.BlowPos, _intake),  # Put intake back up
            # ================= #
            # RETURN TO SPEAKER #
            # ================= #
            PoseReset(_drive),
        )


NAME = "Grab Note"
load = lambda bot: GrabNote(bot.limelight, bot.shooter, bot.intake, bot.robotDrive)
