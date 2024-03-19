import commands2
import wpilib

from subsystems.drivesubsystem import DriveSubsystem


class FlyWithWires(commands2.CommandBase):
    def __init__(
        self, driveSubsystem: DriveSubsystem, xSpeed=0, ySpeed=0, rot=0, time=-1
    ) -> None:
        super().__init__()

        self.driveSubsystem = driveSubsystem
        self.addRequirements((self.driveSubsystem))

        # time and derection vars
        self.time = time
        self.xSpeed = xSpeed
        self.ySpeed = ySpeed
        self.rot = rot

        # Timer
        self.backgroundTimer = wpilib.Timer()
        self.backgroundTimer.start()

        print("initalized")

    def initialize(self) -> None:
        self.backgroundTimer.reset()

    def execute(self) -> None:
        self.driveSubsystem.drive(
            self.xSpeed,
            self.ySpeed,
            self.rot,
            False,
            True,
        )
        print("running")

    def end(self, interrupted: bool) -> None:
        self.driveSubsystem.drive(0, 0, 0, False, True)
        print("ended")

    def isFinished(self) -> bool:
        if self.time != -1 and self.backgroundTimer.hasElapsed(self.time):
            return True
        print("finished")
