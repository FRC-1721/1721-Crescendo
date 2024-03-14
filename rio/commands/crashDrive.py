import commands2
import logging

from subsystems.drivesubsystem import DriveSubsystem

from wpimath.controller import PIDController


class crashDrive(commands2.Command):
    def __init__(
        self,
        _drivetrain=DriveSubsystem,
    ):
        super().__init__()

        # local subsystem instance
        self.subsystem = _drivetrain
        self.addRequirements(self.subsystem)

        self.fixHeadingPID = PIDController(0.004, 0.0, 0.0)

    def initialize(self):
        pass

    def execute(self):
        # Fix our heading with a small pid loop
        yawAdjustment = self.fixHeadingPID.calculate(self.subsystem.getHeading(), 180)
        logging.info(
            f"Calculated yaw adjustment to be {yawAdjustment}, error was {self.subsystem.getHeading()}"
        )

        self.subsystem.drive(-0.2, 0, yawAdjustment, False, False)

    def isFinished(self):
        return self.subsystem.getAcc()

    def end(self, interrupted: bool):
        logging.info("acceleration iz zer0 mee bio")
        return True
