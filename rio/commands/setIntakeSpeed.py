import logging
import commands2

from subsystems.intake import IntakeSubsystem


class SetIntakeSpeed(commands2.Command):
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
        if self.speed > 0:
            if not self.intakeSubsystem.switchPress():
                self.intakeSubsystem.intake(self.speed)
        else:
            self.intakeSubsystem.intake(self.speed)

    def isFinished(self):
        return True

    def end(self, interrupted: bool):
        self.intakeSubsystem.intake(False)
        logging.debug(f"Intake suck done")
        return True
