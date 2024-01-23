import commands2

from subsystems.intake import IntakeSubsystem


class IntakeSuck(commands2.Command):
    def __init__(self, speed):
        """
        takes in the rings
        """
        super().__init__()

        # local subsystem instance
        self.intakeSubsystem = IntakeSubsystem()

        # requested speed
        self.speed = speed

        # TODO change current limit later in amps
        self.intakeSubsystem.intakeCurrentLimit(1)

    def initialize(self):
        self.intakeSubsystem.intake(self.speed)

    def end(self, interrupted: bool):
        self.intakeSubsystem.intake(0)
        return True
