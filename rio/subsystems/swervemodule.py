# vendor libs
from rev import CANSparkMax, SparkMaxAbsoluteEncoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

# constants
from constants.mathConstant import ModuleConstants
from constants.constants import getConstants


class SwerveModule:
    def __init__(self, driveCANId: int, turnCANId: int, chassisAngularOffset: float):
        """
        A Swerve Module
        """
        # constants
        # hardware
        hardwareConstants = getConstants("robot_hardware")
        self.driveconsts = hardwareConstants["driveTrain"]

        # PID
        self.pidConsts = getConstants("robot_pid")["PID"]

        self.chassisAngularOffset = 0
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        self.driveSparkMax = CANSparkMax(driveCANId, CANSparkMax.MotorType.kBrushless)
        self.turnSparkMax = CANSparkMax(turnCANId, CANSparkMax.MotorType.kBrushless)

        # This helps get it to a known state for less tomfoolery
        self.driveSparkMax.restoreFactoryDefaults()
        self.turnSparkMax.restoreFactoryDefaults()

        self.driveEncoder = self.driveSparkMax.getEncoder()
        self.turnEncoder = self.turnSparkMax.getAbsoluteEncoder(
            SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.drivePIDController = self.driveSparkMax.getPIDController()
        self.turnPIDController = self.turnSparkMax.getPIDController()
        self.drivePIDController.setFeedbackDevice(self.driveEncoder)
        self.turnPIDController.setFeedbackDevice(self.turnEncoder)

        # Appling position and velocity conversion factors for driving encoder
        self.driveEncoder.setPositionConversionFactor(
            ModuleConstants.driveEncoderPositionFactor
        )
        self.driveEncoder.setVelocityConversionFactor(
            ModuleConstants.driveEncoderVelocityFactor
        )

        # Appling position and velocity conversion factors for turning encoder
        self.turnEncoder.setPositionConversionFactor(
            ModuleConstants.turnEncoderPositionFactor
        )
        self.turnEncoder.setVelocityConversionFactor(
            ModuleConstants.turnEncoderVelocityFactor
        )

        self.turnEncoder.setInverted(self.driveconsts["turnEncoderInverted"])

        # Enable PID wrapping
        self.turnPIDController.setPositionPIDWrappingEnabled(True)
        self.turnPIDController.setPositionPIDWrappingMinInput(
            ModuleConstants.turnEncoderPositionPIDMinInput
        )
        self.turnPIDController.setPositionPIDWrappingMaxInput(
            ModuleConstants.turnEncoderPositionPIDMaxInput
        )

        # PIDS
        self.drivePIDController.setP(self.pidConsts["driveP"])
        self.drivePIDController.setI(self.pidConsts["driveI"])
        self.drivePIDController.setD(self.pidConsts["driveD"])
        self.drivePIDController.setFF(self.pidConsts["driveFF"])
        self.drivePIDController.setOutputRange(
            self.pidConsts["driveMinOutput"], self.pidConsts["driveMaxOutput"]
        )

        self.turnPIDController.setP(self.pidConsts["turnP"])
        self.turnPIDController.setI(self.pidConsts["turnI"])
        self.turnPIDController.setD(self.pidConsts["turnD"])
        self.turnPIDController.setFF(self.pidConsts["turnFF"])
        self.turnPIDController.setOutputRange(
            self.pidConsts["turnMinOutput"], self.pidConsts["turnMaxOutput"]
        )

        self.driveSparkMax.setIdleMode(ModuleConstants.driveMotorIdleMode)
        self.turnSparkMax.setIdleMode(ModuleConstants.turnMotorIdleMode)
        self.driveSparkMax.setSmartCurrentLimit(
            self.driveconsts["driveMotorCurrentLimit"]
        )
        self.turnSparkMax.setSmartCurrentLimit(
            self.driveconsts["turnMotorCurrentLimit"]
        )

        # Save the SPARK MAX configs
        self.driveSparkMax.burnFlash()
        self.turnSparkMax.burnFlash()

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState.angle = Rotation2d(self.turnEncoder.getPosition())
        self.driveEncoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """
        This is relative to the chassis
        """
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.turnEncoder.getPosition() - self.chassisAngularOffset),
        )

    def getPosition(self) -> SwerveModulePosition:
        """
        This is relative to the chassis
        """
        return SwerveModulePosition(
            self.driveEncoder.getPosition(),
            Rotation2d(self.turnEncoder.getPosition() - self.chassisAngularOffset),
        )

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """
        sets a desired state
        """
        # add angular offset
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(
            self.chassisAngularOffset
        )

        # Optimizeation
        optimizedDesiredState = SwerveModuleState.optimize(
            correctedDesiredState, Rotation2d(self.turnEncoder.getPosition())
        )

        # got to setpoints
        self.drivePIDController.setReference(
            optimizedDesiredState.speed, CANSparkMax.ControlType.kVelocity
        )
        self.turnPIDController.setReference(
            optimizedDesiredState.angle.radians(), CANSparkMax.ControlType.kPosition
        )

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        """
        '-+es all the SwerveModule encoders.' - rev
        """
        self.driveEncoder.setPosition(0)
