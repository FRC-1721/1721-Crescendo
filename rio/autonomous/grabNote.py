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
from commands.SendToPos import sendToFieldPos


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
        self.limelight = _limelight
        self.drivetrain = _drive
        self.shooter = _shooter
        self.intake = _intake
        super().__init__(
            # Resets Yaw relative to the robot's starting position
            ResetYaw(_drive),
            # ============ #
            # SPEAKER SHOT #
            # ============ #
            RotateIntake(
                0, self.intake
            ),  # Put intake fully inside (if it wasn't already)
            FlyWheelSpeed(1.0, self.shooter, False),  # Power up the flywheels (?)
            SetIntakeSpeed(-0.6, self.intake),  # Load magazine? (but without ending)
            commands2.WaitCommand(1),
            FlyWheelSpeed(0.0, self.shooter),  # Stop flywheel
            SetIntakeSpeed(0, self.intake),  # Stop intake
            # ================ #
            # COLLECT NEW NOTE #
            # ================ #
            sendToObject(self.drivetrain, self.limelight),
            ShooterROT(SuperStrucConstants.LoadPos, self.shooter),
            FlyWheelSpeed(0, self.shooter),  # Stop shooter (if its running)
            RotateIntake(
                122.5, self.intake
            ),  # Put intake down (with a lil extra squeeze)
            intakeUntilNote(0.5, self.intake),  # Intake till note
            RotateIntake(0, self.intake),  # Put intake back up
            # ================= #
            # RETURN TO SPEAKER #
            # ================= #
            sendToFieldPos(
                0.9245,
                2.117,
            ),
        )


NAME = "Get Note"
load = lambda bot: GrabNote(bot.limelight, bot.shooter, bot.intake, bot.robotDrive)
