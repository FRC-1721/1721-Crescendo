import commands2

from subsystems.intake import IntakeSubsystem


class IntakeRotationPID(commands2.Command):
    def __init__(self, angle, subsystem):
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
        self.intakeSubsystem.lift(self.angle)
        print("HELP")

    def end(self, interrupted: bool):
        print("END")
        return True
