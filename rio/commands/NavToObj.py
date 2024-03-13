import commands2
import logging

from subsystems.limelight import limeLightCommands
from subsystems.drivesubsystem import DriveSubsystem


class sendToObject(commands2.Command):
    def __init__(self, drive: DriveSubsystem, subsystem: limeLightCommands):
        """
        Sends the robot to a note that it detects
        """
        super().__init__()

        # local subsystem instance
        self.Limelight = subsystem
        self.drive = drive
        self.addRequirements(self.drive)

    def initialize(self):
        self.Limelight.setPipeline(1)  # sets the Limelight to the Detector pipeline

    def execute(self):
        if self.Limelight.findObj() == True:
            self.Limelight.goToObj(self.drive)
            logging.info("Navigating to note")
        else:
            logging.warn("No note detected!")

    def isFinished(self) -> bool:
        return self.Limelight.isAtOBJ()

    def end(self, inturrupted: bool):
        logging.info("Note ready for pickup")
        return True
