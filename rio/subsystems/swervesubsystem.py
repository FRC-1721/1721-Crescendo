# rename to drivesubsystem.py when completed
# replace IMU with our gyro

from commands2 import Subsystem
from constants.constants import getConstants
from .swerveModule import SwerveModule
from wpilib import Timer
from wpimath.filter import SlewRateLimiter


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
        self.gyro = ADIS16448_IMU()

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(self.driveConst["MagnitudeSlewRate"])
        self.rotLimiter = SlewRateLimiter(self.driveConst["RotationalSlewRate"])

        # timer
        self.prevTime = Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.rearLeft.getPosition(),
                self.rearRight.getPosition(),
            ),
        )
