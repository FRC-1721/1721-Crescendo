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

    def execute(self):
        if self.speed > 0:
            if not self.intakeSubsystem.switchPress():
                commands2.waitcommand(0.25)
                self.intakeSubsystem.intake(self.speed)

        else:
            self.intakeSubsystem.intake(self.speed)

    def end(self, interrupted: bool):
        self.intakeSubsystem.intake(0)
        return True
