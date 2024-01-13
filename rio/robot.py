# vendor libs
import commands2
import wpilib

# robot container
from robotcontainer import RobotContainer


class UnnamedToaster(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer()
        self.autonomousCommand = None

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(UnnamedToaster)
