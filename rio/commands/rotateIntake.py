import commands2
import logging

from subsystems.intake import IntakeSubsystem


class RotateIntake(commands2.Command):
    def __init__(self, angle: float, subsystem: IntakeSubsystem):
        """
        rotates the intake
        supposed to be used with
        presets
        """
        super().__init__()

        # local subsystem instance
        self.intakeSubsystem = subsystem

        # requested speed
        self.angle = angle

    def initialize(self):
        logging.info(f"Moving intake to {self.angle}")

    def execute(self):
        self.intakeSubsystem.lift(self.angle)
        logging.info(
            f"Still moving... {self.intakeSubsystem.getAngle()} to {self.angle}"
        )

    def isFinished(self):
        e = 1  # How close before we're done

        return abs(self.intakeSubsystem.getAngle() - self.angle) < e

    def end(self, interrupted: bool):
        logging.info(f"Done moving! Finished normally: {interrupted}")
        return True
