import logging
import commands2

from subsystems.intake import IntakeSubsystem


class intakeUntilNote(commands2.Command):
    def __init__(self, speed: float, subsystem: IntakeSubsystem):
        """
        takes in the rings
        """
        super().__init__()

        # local subsystem instance
        self.intakeSubsystem = subsystem

        # requested speed
        self.speed = speed

    def initialize(self):
        logging.debug(f"Running command Intake Suck (manual) with speed {self.speed}")

    def execute(self):
        self.intakeSubsystem.intake(self.speed)
        print(self.intakeSubsystem.switchPress)

    def isFinished(self):
        return self.intakeSubsystem.switchPress()

    def end(self, interrupted: bool):
        self.intakeSubsystem.intake(0)
        logging.debug(f"Intake suck done")
        return True
