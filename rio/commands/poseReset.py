import commands2
import logging

from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d


class PoseReset(commands2.Command):
    def __init__(
        self,
        _drivetrain=DriveSubsystem,
    ):
        super().__init__()

        # local subsystem instance
        self.drivetrain = _drivetrain

    def initialize(self):
        # what do I put here? :3
        pass

    def execute(self):
        driveX = 0
        driveY = 0
        rotZ = 0
        Pose = self.drivetrain.getPose()

        if Pose.X() > 0:
            driveX = 0.1
        elif Pose.X() < 0:
            driveX = -0.1

        if Pose.Y() > 0:
            driveY = 0.1
        elif Pose.Y() < 0:
            driveY = -0.1

        if Pose.rotation().degrees() > 0:
            rotZ = 0.1
        elif Pose.rotation().degrees() < 0:
            rotZ = -0.1

        self.drivetrain.drive(driveX, driveY, rotZ, True, True)

    def isFinished(self):
        return self.drivetrain.isPoseZero()

    def end(self, interrupted: bool):
        logging.info("Yaw iz zero mah boi")
        return True
