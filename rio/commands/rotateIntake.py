import commands2
import logging

from subsystems.intake import IntakeSubsystem


class RotateIntake(commands2.Command):
    def __init__(self, angle: float, _intake: IntakeSubsystem):
        """
        rotates the intake
        supposed to be used with
        presets
        """
        super().__init__()

        # local subsystem instance
        self.intakeSubsystem = _intake

        # requested speed
        self.angle = angle

        # Command requirements
        self.addRequirements(self.intakeSubsystem)

    def initialize(self):
        logging.info(f"Moving intake to {self.angle}")

    def execute(self):
        if self.intakeSubsystemb.getAngle() < 0:
            # Prevents starting PID from a pose less than 0
            self.intakeSubsystem.manualLift(0.1)
            logging.warn("Intake angle is too inset in frame!")
        else:
            self.intakeSubsystem.lift(self.angle)
            logging.debug(
                f"Still moving... {self.intakeSubsystem.getAngle()} to {self.angle}"
            )

    def isFinished(self):
        e = 1  # How close before we're done

        return abs(self.intakeSubsystem.getAngle() - self.angle) < e

    def end(self, interrupted: bool):
        if not interrupted:
            logging.info(f"Done intake moved to {self.intakeSubsystem.getAngle()}")
        else:
            logging.warn("RotateIntake was interrupted!")
        return True
