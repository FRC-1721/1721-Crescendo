import commands2
import logging

from subsystems.drivesubsystem import DriveSubsystem
from ntcore import NetworkTableInstance


class ResetYaw(commands2.Command):
    def __init__(
        self,
        _drivetrain=DriveSubsystem,
        angle=0,
    ):
        super().__init__()

        # Configure networktables
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # local subsystem instance
        self.subsystem = _drivetrain

    def initialize(self):
        print("i b setn' 0 t' " + str(self.sd.getNumber("Auto/Angle", 1)))
        self.subsystem.resetYaw(self.sd.getNumber("Auto/Angle", 1))

    def execute(self):
        pass

    def isFinished(self):
        return True

    def end(self, interrupted: bool):
        logging.info("Yaw iz zero mah boi")
        return True
