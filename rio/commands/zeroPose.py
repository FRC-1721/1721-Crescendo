import commands2
import logging

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Pose2d, Rotation2d


class zeroPose(commands2.Command):
    def __init__(
        self,
        _drivetrain=DriveSubsystem,
    ):
        super().__init__()

        # local subsystem instance
        self.subsystem = _drivetrain

    def initialize(self):
        self.subsystem.resetOdometry(Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0)))

    def execute(self):
        pass

    def isFinished(self):
        return True

    def end(self, interrupted: bool):
        logging.info("Pose iz zero mah boi")
        return True
