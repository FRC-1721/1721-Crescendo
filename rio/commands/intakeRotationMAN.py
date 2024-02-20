import commands2

from subsystems.intake import IntakeSubsystem


class IntakeRotationMAN(commands2.Command):
    def __init__(self, angle: float, subsystem):
        """
        allows people to rotate the intake
        """
        super().__init__()

        # local subsystem instance
        self.intakeSubsystem = subsystem

        # requested speed
        self.angle = angle

    def execute(self):
        self.intakeSubsystem.manualLift(self.angle)

    def end(self, interrupted: bool):
        self.intakeSubsystem.manualLift(0)
        return True
