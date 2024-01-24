import commands2

from subsystems.intake import IntakeSubsystem


class IntakeRotation(commands2.Command):
    def __init__(self, angle):
        """
        rotates the intake
        """
        super().__init__()

        # local subsystem instance
        self.intakeSubsystem = IntakeSubsystem()

        # requested speed
        self.angle = angle

        # TODO change current limit later in amps
        self.intakeSubsystem.liftCurrentLimit(1)

    def initialize(self):
        self.intakeSubsystem.lift(self.angle)

    def end(self, interrupted: bool):
        self.intakeSubsystem.lift(0)
        return True
