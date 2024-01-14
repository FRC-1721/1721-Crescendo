#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# math
import math

# wpilib
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

# rev
from rev import CANSparkMaxLowLevel, CANSparkMax

kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        driveEncoderChannelA: int,
        driveEncoderChannelB: int,
        turningEncoderChannelA: int,
        turningEncoderChannelB: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        :param driveEncoderChannelA:   DIO input for the drive encoder channel A
        :param driveEncoderChannelB:   DIO input for the drive encoder channel B
        :param turningEncoderChannelA: DIO input for the turning encoder channel A
        :param turningEncoderChannelB: DIO input for the turning encoder channel B
        """
        self.driveMotor = CANSparkMax(
            driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.turningMotor = CANSparkMax(
            turningMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless
        )

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = self.driveMotor.getPIDController()

        self.drivePIDController.setP(0)
        self.drivePIDController.setI(0)
        self.drivePIDController.setD(0)
        self.drivePIDController.setFF(0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = self.turningMotor.getPIDController()

        self.turningPIDController.setP(0)
        self.turningPIDController.setI(0)
        self.turningPIDController.setD(0)
        self.turningPIDController.setFF(0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.
        self.driveEncoder.setPositionConversionFactor(
            (math.tau * kWheelRadius / kEncoderResolution)
        )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.
        self.turningEncoder.setPositionConversionFactor((math.tau / kEncoderResolution))

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getVelocity()),
        )

    def aquirePosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getVelocity()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getVelocity())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getPosition(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getVelocity(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)
