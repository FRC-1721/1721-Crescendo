import commands2

from subsystems.intake import IntakeSubsystem


class IntakeSuck(commands2.Command):
    def __init__(self, speed: float, subsystem: IntakeSubsystem):
        """
        takes in the rings
        """
        super().__init__()

        # local subsystem instance
        self.intakeSubsystem = subsystem

        # requested speed
        self.speed = speed

        # TODO change current limit later in amps
        self.intakeSubsystem.intakeCurrentLimit(30)

    def execute(self):
        self.intakeSubsystem.intake(self.speed)

    def end(self, interrupted: bool):
        self.intakeSubsystem.intake(0)
        return True
