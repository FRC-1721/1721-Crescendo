import commands2

from subsystems.drivesubsystem import DriveSubsystem
class ResetYaw(commands2.Command):
    def __init__(self, _drivetrain = DriveSubsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = _drivetrain

    def initialize(self):
        self.subsystem.resetYaw()

    def end(self, interrupted: bool):
        return True
