import commands2
import logging

from subsystems.drivesubsystem import DriveSubsystem


class ResetYaw(commands2.Command):
    def __init__(
        self,
        _drivetrain=DriveSubsystem,
        angle=0,
    ):
        super().__init__()

        # local subsystem instance
        self.subsystem = _drivetrain
        self.angle = angle

    def initialize(self):
        self.subsystem.resetYaw(self.angle)

    def execute(self):
        pass

    def isFinished(self):
        return True

    def end(self, interrupted: bool):
        logging.info("Yaw iz zero mah boi")
        return True
