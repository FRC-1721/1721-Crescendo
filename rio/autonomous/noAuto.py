import commands2
from subsystems.drivesubsystem import DriveSubsystem
from commands.ResetYaw import ResetYaw


class NoAuto(commands2.SequentialCommandGroup):
    def __init__(self, _drive: DriveSubsystem):
        """
        This doesn't do anything besides
        reset yaw
        """
        super().__init__(ResetYaw(_drive, 180))


NAME = "No Auto"
load = lambda bot: NoAuto(bot.robotDrive)
