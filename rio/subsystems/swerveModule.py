# vendor libs
from rev import CANSparkMax, CANSparkMaxLowLevel, SparkMaxAbsoluteEncoder

from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d

# constants
from constants.constants import getConstants


class SwerveModule:
    def __init__(
        self,
        driveChannel: int,
        turnChannel: int,
        chassisAngularOffset: float,
    ) -> None:
        """
        LMAO look at this joke code
        it 'claims' to be a swerve module

        driveChannel: drive motor id
        turnChannel:  turn motor id
        driveEncoder: encoder channel
        turnEncoder:  encoder channel
        """

        # constants
        constants = getConstants("robot_hardware")
        self.driveConstants = constants["drivetrain"]
        self.drivePID = self.driveConstants["DrivePIDValues"]
        self.turnPID = self.driveConstants["TurnPIDValues"]
        conversions = constants["Conversions"]
        self.driveConversions = conversions["Drive"]
        self.turnConversions = conversions["Turn"]

        # motors
        self.driveMotor = CANSparkMax(
            driveChannel, CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.turnMotor = CANSparkMax(
            turnChannel, CANSparkMaxLowLevel.MotorType.kBrushless
        )

        # factory reset so we know whats been done to them
        self.driveMotor.restoreFactoryDefaults()
        self.turnMotor.restoreFactoryDefaults()

        # encoders
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turnEncoder = self.turnMotor.getAbsoluteEncoder(
            SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )

        # converting units to meters and meters per second to better work with WPILib's swerve APIs
        self.driveEncoder.setPositionConversionFactor(
            self.driveConversions["MotorPositionFactor"]
        )
        self.driveEncoder.setVelocityConversionFactor(
            self.driveConversions["EncoderVelocityFactor"]
        )

        self.turnEncoder.setPositionConversionFactor(
            self.turnConversions["EncoderPositionFactor"]
        )
        self.turnEncoder.setVelocityConversionFactor(
            self.turnConversions["EncoderVelocityFactor"]
        )

        # we may need to invert the turning encoder
        # self.turnEncoder.setInverted(True)

        # PID
        self.drivePIDController = self.driveMotor.getPIDController()
        self.turnPIDController = self.turnMotor.getPIDController()

        # feedback
        self.drivePIDController.setFeedbackDevice(self.driveEncoder)
        self.turnPIDController.setFeedbackDevice(self.turnEncoder)

        # drive PIDS
        self.drivePIDController.setP(self.drivePID["P"])
        self.drivePIDController.setI(self.drivePID["I"])
        self.drivePIDController.setD(self.drivePID["D"])
        self.drivePIDController.setFF(self.drivePID["FF"])
        self.drivePIDController.setOutputRange(
            self.drivePID["MinOutput"], self.drivePID["MaxOutput"]
        )

        # turn PIDS
        self.turnPIDController.setP(self.turnPID["P"])
        self.turnPIDController.setI(self.turnPID["I"])
        self.turnPIDController.setD(self.turnPID["D"])
        self.turnPIDController.setFF(self.turnPID["FF"])
        self.turnPIDController.setOutputRange(
            self.turnPID["MinOutput"], self.turnPID["MaxOutput"]
        )

        # this should fix it spinning from 359 to zero the long way
        self.turnPIDController.setPositionPIDWrappingEnabled(True)
        self.turnPIDController.setPositionPIDWrappingMinInput(
            self.turnConversions["EncoderPositionPIDMinInput"]
        )
        self.turnPIDController.setPositionPIDWrappingMaxInput(
            self.turnConversions["EncoderPositionPIDMaxInput"]
        )

        # defining idle modes
        self.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)

        # setting limits
        self.driveMotor.setSmartCurrentLimit(
            self.driveConstants["DriveMotorCurrentLimt"]
        )
        self.turnMotor.setSmartCurrentLimit(self.driveConstants["TurnMotorCurrentLimt"])

        # save config, to ensure config says after brown outs
        self.driveMotor.burnFlash()
        self.turnMotor.burnFlash()

        # Chassis angular offset
        self.chassisAngularOffset = 0
        self.chassisAngularOffset = chassisAngularOffset

        # making the desired state
        self.desiredState = SwerveModuleState(0.0, Rotation2d())
        self.desiredState.angle = Rotation2d(self.turnEncoder.getPosition())

        # misc
        self.driveEncoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """
        returns the current state
        this is all relative to the chassis
        """

        # applies angular offset of chassis to encoder position to get a final position
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.turnEncoder.getPosition() - self.chassisAngularOffset),
        )

    def getPosition(self) -> SwerveModulePosition:
        """
        returns the position
        this is all relative to the chassis
        """

        # applies angular offset of chassis to encoder position to get a final position
        return SwerveModulePosition(
            self.driveEncoder.getPosition(),
            Rotation2d(self.turnEncoder.getPosition() - self.chassisAngularOffset),
        )

    def setDesiredState(self, desiredState: SwerveModuleState):
        """
        set a new desired state
        """

        # add chassis offset
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(
            self.chassisAngularOffset
        )

        # make it avoid spinning over 90 in reference state
        optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d(self.turnEncoder.getPosition())
        )

        # turn to setpoints
        self.drivePIDController.setReference(
            optimizedDesiredState.speed, CANSparkMax.ControlType.kVelocity
        )
        self.turnPIDController.setReference(
            optimizedDesiredState.angle.radians(), CANSparkMax.ControlType.kPosition
        )

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        """
        to quote rev as to the function of this
        "-+es all the SwerveModule encoders."
        """
        self.driveEncoder.setPosition(0)
