import commands2

from subsystems.drivesubsystem import DriveSubsystem


class ResetYaw(commands2.Command):

    def __init__(
        self,
        _drivetrain=DriveSubsystem,
        angle=0,
    ):
        super().__init__()

        # local subsystem instance
        self.subsystem = _drivetrain
        self.angle = angle

    def initialize(self):
        self.subsystem.resetYaw(self.angle)

    def end(self, interrupted: bool):
        return True
