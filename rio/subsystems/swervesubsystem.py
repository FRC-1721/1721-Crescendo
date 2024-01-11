# rename to drivesubsystem.py when completed

from commands2 import Subsystem
from constants.constants import getConstants
from .swerveModule import SwerveModule
from wpilib import Timer
from wpimath.filter import SlewRateLimiter
from navx import AHRS
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.geometry import Rotation2d


class drivesubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # constants
        constants = getConstants("robot_hardware")
        self.driveConst = constants["drivetrain"]

        # module constants
        self.fPMotorsConst = self.driveConst["FPMotors"]
        self.fSMotorsConst = self.driveConst["FSMotors"]
        self.aPMotorsConst = self.driveConst["APMotors"]
        self.aSMotorsConst = self.driveConst["ASMotors"]

        # defining the modules
        self.fPModule = SwerveModule(
            self.fPMotorsConst["drivePort"],
            self.fPMotorsConst["turnPort"],
            self.fPMotorsConst["ChassisAngularOffset"],
        )
        self.fSModule = SwerveModule(
            self.fSMotorsConst["drivePort"],
            self.fSMotorsConst["turnPort"],
            self.fSMotorsConst["ChassisAngularOffset"],
        )
        self.aPModule = SwerveModule(
            self.aPMotorsConst["drivePort"],
            self.aPMotorsConst["turnPort"],
            self.aPMotorsConst["ChassisAngularOffset"],
        )
        self.aSModule = SwerveModule(
            self.aSMotorsConst["drivePort"],
            self.aSMotorsConst["turnPort"],
            self.aSMotorsConst["ChassisAngularOffset"],
        )

        # The gyro sensor
        self.gyro = AHRS.create_spi()

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(self.driveConst["MagnitudeSlewRate"])
        self.rotLimiter = SlewRateLimiter(self.driveConst["RotationalSlewRate"])

        # timer
        self.prevTime = Timer.getFPGATimestamp()

        # for taking robit POS
        # TODO start with this tomorrow
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.fPModule.getPosition(),
                self.fSModule.getPosition(),
                self.aPModule.getPosition(),
                self.aSModule.getPosition(),
            ),
        )

    # TODO rest goes here

    def periodic(self) -> None:
        # update the robot pos
        self.odometry.update(
            Rotation2d.fromDegrees(self.gyro.getPitch()),
            (
                self.fPModule.getPosition(),
                self.fSModule.getPosition(),
                self.aPModule.getPosition(),
                self.aSModule.getPosition(),
            ),
        )
