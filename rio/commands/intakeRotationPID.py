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

        # TODO change current limit later in amps
        self.intakeSubsystem.liftCurrentLimit(1)

    def initialize(self):
        self.intakeSubsystem.lift(self.angle)

    def end(self, interrupted: bool):
        self.intakeSubsystem.lift(0)
        return True
