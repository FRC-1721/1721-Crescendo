import commands2

from subsystems.limelight import limeLightCommands
from subsystems.drivesubsystem import DriveSubsystem


class sendToFieldPos(commands2.Command):
    def __init__(
        self,
        X: int,
        Y: int,
        Z: int,
        drive: DriveSubsystem,
        subsystem: limeLightCommands,
    ):
        """
        Sends the robot to a specified location
        """
        super().__init__()

        # local subsystem instance
        self.Limelight = subsystem
        self.distX = X
        self.distY = Y
        self.rotZ = Z
        self.drive = drive

    def initialize(self):
        self.Limelight.setPipeline(0)  # sets the Limelight to the apriltag pipeline

    def execute(self):
        self.Limelight.getPoseInField()  # robot uses apriltags to figure out where it is relative to the field

        self.Limelight.sendToPos(
            self.distX, self.distY, self.rotZ, self.drive
        )  # Sends the robot to the desired location

    def end(self):
        pass
        return True
