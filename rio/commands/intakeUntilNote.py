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
        if not self.intakeSubsystem.switchPress():
            logging.info(
                f"Running command Intake Suck (manual) with speed {self.speed}"
            )
        else:
            logging.warn("Switch is jammed at start of command!")

    def execute(self):
        self.intakeSubsystem.intake(self.speed)
        logging.debug(self.intakeSubsystem.switchPress)

    def isFinished(self):
        return self.intakeSubsystem.switchPress()

    def end(self, interrupted: bool):
        self.intakeSubsystem.intake(0)
        if not interrupted:
            logging.info(f"Intake suck done")
        else:
            logging.warn("intakeUntilNote was interrupted!")
        return True
