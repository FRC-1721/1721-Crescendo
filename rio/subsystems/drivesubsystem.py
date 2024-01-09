# Robot
import wpilib
import wpilib.drive
import commands2
from ntcore import NetworkTableInstance

from wpimath.geometry import Pose2d
from wpimath.kinematics import DifferentialDriveOdometry
from wpilib import Field2d

# Constants
from constants.constants import getConstants

# Vendor Libs
from rev import CANSparkMax, CANSparkMaxLowLevel
from navx import AHRS


class DriveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # Configure networktables
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # network tables
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # Hardware consts
        constants = getConstants("robot_hardware")
        self.driveConst = constants["drivetrain"]  # All the drivetrain consts
        self.leftCosnt = self.driveConst["leftMotor"]  # Left specific
        self.rightCosnt = self.driveConst["rightMotor"]  # Right specific

        # The motors on the left side of the drive.
        self.leftMotor1 = CANSparkMax(
            self.leftCosnt["Motor1Port"],
            CANSparkMaxLowLevel.MotorType.kBrushless,
        )
        self.leftMotor2 = CANSparkMax(
            self.leftCosnt["Motor2Port"],
            CANSparkMaxLowLevel.MotorType.kBrushless,
        )

        # Combine left motors into one group
        self.leftMotors = wpilib.MotorControllerGroup(
            self.leftMotor1,
            self.leftMotor2,
        )
        self.leftMotors.setInverted(self.leftCosnt["Inverted"])

        # The motors on the right side of the drive.
        self.rightMotor1 = CANSparkMax(
            self.rightCosnt["Motor1Port"],
            CANSparkMaxLowLevel.MotorType.kBrushless,
        )
        self.rightMotor2 = CANSparkMax(
            self.rightCosnt["Motor2Port"],
            CANSparkMaxLowLevel.MotorType.kBrushless,
        )

        # Combine left motors into one group
        self.rightMotors = wpilib.MotorControllerGroup(
            self.rightMotor1,
            self.rightMotor2,
        )
        self.rightMotors.setInverted(self.rightCosnt["Inverted"])
        # self.rightMotors.setInverted(False)

        # This fixes a bug in rev firmware involving flash settings.
        self.leftMotor1.setInverted(False)
        self.leftMotor2.setInverted(False)
        self.rightMotor1.setInverted(False)
        self.rightMotor2.setInverted(False)

        # The robot's drivetrain kinematics
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotors, self.rightMotors)

        # The left-side drive encoder
        self.leftEncoder = self.leftMotor1.getEncoder()

        # The right-side drive encoder
        self.rightEncoder = self.rightMotor2.getEncoder()

        # PID Controllers
        self.lPID = self.leftMotor1.getPIDController()
        self.rPID = self.rightMotor1.getPIDController()

        # Setup the conversion factors for the motor controllers
        # TODO: Because rev is rev, there are a lot of problems that need to be addressed.
        # https://www.chiefdelphi.com/t/spark-max-encoder-setpositionconversionfactor-not-doing-anything/396629
        self.leftEncoder.setPositionConversionFactor(
            1 / self.driveConst["encoderConversionFactor"]
        )
        self.rightEncoder.setPositionConversionFactor(
            1 / self.driveConst["encoderConversionFactor"]
        )

        # Gyro
        self.ahrs = AHRS.create_spi()  # creates navx object

        # Robot odometry
        self.field = Field2d()
        self.odometry = DifferentialDriveOdometry(
            self.ahrs.getRotation2d(),
            self.leftEncoder.getPosition(),
            self.rightEncoder.getPosition(),
        )

        # Enable braking
        self.rightMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.rightMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.leftMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.leftMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake)

    def arcadeDrive(self, fwd: float, rot: float):
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(fwd, rot)

    def tankDriveVolts(self, leftVolts, rightVolts):
        """Control the robot's drivetrain with voltage inputs for each side."""
        # Set the voltage of the left side.
        # inverting this delays the KP issue but doesn't fix it
        self.leftMotors.setVoltage(leftVolts)

        # Set the voltage of the right side.
        self.rightMotors.setVoltage(rightVolts)

        # print(f"({leftVolts}, {rightVolts})")

        # Resets the timer for this motor's MotorSafety
        self.drive.feed()

        # Reset the encoders
        self.resetEncoders()

    def setMaxOutput(self, maxOutput: float):
        """
        Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
        :param maxOutput: the maximum output to which the drive will be constrained
        """
        self.drive.setMaxOutput(maxOutput)

    def resetEncoders(self):
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)

        # https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html#resetting-the-robot-pose
        self.odometry.resetPosition(
            self.ahrs.getRotation2d(),
            self.leftEncoder.getPosition(),
            -self.rightEncoder.getPosition(),
            Pose2d(),
        )
