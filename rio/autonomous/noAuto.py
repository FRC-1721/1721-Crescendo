import commands2
from subsystems.drivesubsystem import DriveSubsystem
from commands.ResetYaw import ResetYaw


class NoAuto(commands2.SequentialCommandGroup):
    def __init__(self, _drive: DriveSubsystem):
        self.drivetrain = _drive
        super().__init__(ResetYaw(0))


NAME = "No Auto"
load = lambda bot: NoAuto(bot.robotDrive)
