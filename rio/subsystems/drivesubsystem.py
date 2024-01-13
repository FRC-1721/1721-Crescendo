# wpilib
import wpilib

from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

# swerve stuff
import swerveutils
from .swervemodule import SwerveModule

# constants
from constants.constants import getConstants
from constants.mathConstant import DriveConstants

# misc
import math

from commands2 import Subsystem
from typing import Tuple


class DriveSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # constants
        constants = getConstants("robot_hardware")
        self.driveConsts = constants["driveTrain"]
        self.autoConsts = self.driveConsts["autonomous"]

        # module constants
        self.fPConsts = self.driveConsts["FPMotors"]
        self.fSConsts = self.driveConsts["FSMotors"]
        self.aPConsts = self.driveConsts["APMotors"]
        self.aSConsts = self.driveConsts["ASMotors"]

        # Create SwerveModules
        self.fPModule = SwerveModule(
            self.fPConsts["driveID"],
            self.fPConsts["turnID"],
            DriveConstants.fPChassisAngularOffset,
        )

        self.fSModule = SwerveModule(
            self.fSConsts["driveID"],
            self.fSConsts["turnID"],
            DriveConstants.fSChassisAngularOffset,
        )

        self.aPModule = SwerveModule(
            self.aPConsts["driveID"],
            self.aPConsts["turnID"],
            DriveConstants.aPChassisAngularOffset,
        )

        self.aSModule = SwerveModule(
            self.aSConsts["driveID"],
            self.aSConsts["turnID"],
            DriveConstants.aSChassisAngularOffset,
        )

        # The gyro sensor
        self.gyro = wpilib.ADIS16448_IMU()

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(self.driveConsts["MagnitudeSlewRate"])
        self.rotLimiter = SlewRateLimiter(self.driveConsts["RotationalSlewRate"])
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.driveKinematics,
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.fPModule.getPosition(),
                self.fSModule.getPosition(),
                self.aPModule.getPosition(),
                self.aSModule.getPosition(),
            ),
        )

    def periodic(self) -> None:
        # Update the odometry in the periodic block
        self.odometry.update(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.fPModule.getPosition(),
                self.fSModule.getPosition(),
                self.aPModule.getPosition(),
                self.aSModule.getPosition(),
            ),
        )

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            (
                self.fPModule.getPosition(),
                self.fSModule.getPosition(),
                self.aPModule.getPosition(),
                self.aSModule.getPosition(),
            ),
            pose,
        )

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        rateLimit: bool,
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """

        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if rateLimit:
            # Convert XY to polar for rate limiting
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.hypot(xSpeed, ySpeed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(
                    self.driveConsts["DirectionSlewRate"] / self.currentTranslationMag
                )
            else:
                directionSlewRate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            currentTime = wpilib.Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.prevTime
            angleDif = swerveutils.angleDifference(
                inputTranslationDir, self.currentTranslationDir
            )
            if angleDif < 0.45 * math.pi:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(
                    inputTranslationMag
                )

            elif angleDif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.currentTranslationMag > 1e-4:
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)
                else:
                    self.currentTranslationDir = swerveutils.wrapAngle(
                        self.currentTranslationDir + math.pi
                    )
                    self.currentTranslationMag = self.magLimiter.calculate(
                        inputTranslationMag
                    )

            else:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(0.0)

            self.prevTime = currentTime

            xSpeedCommanded = self.currentTranslationMag * math.cos(
                self.currentTranslationDir
            )
            ySpeedCommanded = self.currentTranslationMag * math.sin(
                self.currentTranslationDir
            )
            self.currentRotation = self.rotLimiter.calculate(rot)

        else:
            self.currentRotation = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * self.autoConsts["maxSpeedMetersPerSecond"]
        ySpeedDelivered = ySpeedCommanded * self.autoConsts["maxSpeedMetersPerSecond"]
        rotDelivered = self.currentRotation * DriveConstants.maxAngularSpeed

        swerveModuleStates = DriveConstants.driveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(self.gyro.getAngle()),
            )
            if fieldRelative
            else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, self.autoConsts["maxSpeedMetersPerSecond"]
        )
        self.fPModule.setDesiredState(fl)
        self.fSModule.setDesiredState(fr)
        self.aPModule.setDesiredState(rl)
        self.aSModule.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.fPModule.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.fSModule.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.aPModule.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.aSModule.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(
        self,
        desiredStates: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, self.autoConsts["maxSpeedMetersPerSecond"]
        )
        self.fPModule.setDesiredState(fl)
        self.fSModule.setDesiredState(fr)
        self.aPModule.setDesiredState(rl)
        self.aSModule.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.fPModule.resetEncoders()
        self.aPModule.resetEncoders()
        self.fSModule.resetEncoders()
        self.aSModule.resetEncoders()

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self) -> float:
        """Returns the heading of the robot.

        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if self.driveConsts["GyroReversed"] else 1.0)
