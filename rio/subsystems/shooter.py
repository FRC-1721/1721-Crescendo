import wpilib
import rev

from ntcore import NetworkTableInstance
from commands2 import Subsystem
from rev import (
    CANSparkLowLevel,
    CANSparkMax,
    SparkAbsoluteEncoder,
    SparkMaxLimitSwitch,
    CANSparkBase,
)

from constants import SuperStrucConstants


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # dashboard
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # motors
        self.rotateMotor = CANSparkMax(
            SuperStrucConstants.rotateID, CANSparkLowLevel.MotorType.kBrushless
        )
        self.flyMotor = CANSparkMax(
            SuperStrucConstants.flyID, CANSparkLowLevel.MotorType.kBrushless
        )

        # encoders
        self.flyEncoder = self.flyMotor.getEncoder()

        self.rotateEncoder = self.rotateMotor.getAbsoluteEncoder(
            SparkAbsoluteEncoder.Type.kDutyCycle
        )

        self.rotateEncoder.setPositionConversionFactor(
            SuperStrucConstants.kRotConversion
        )

        self.rotateEncoder.setInverted(SuperStrucConstants.krotateInversion)

        # self.rotateEncoder.setZeroOffset(1 - 0.3482)  # F*cking love rev sometimes

        # PID values
        # rotate pid
        self.rotatePIDController = self.rotateMotor.getPIDController()
        self.rotatePIDController.setFeedbackDevice(self.rotateEncoder)
        self.rotatePIDController.setPositionPIDWrappingEnabled(False)

        self.rotatePIDController.setP(SuperStrucConstants.krotateP)
        self.rotatePIDController.setI(SuperStrucConstants.krotateI)
        self.rotatePIDController.setD(SuperStrucConstants.krotateD)
        self.rotatePIDController.setFF(SuperStrucConstants.krotateFF)

        # fly pid
        self.flyPIDController = self.flyMotor.getPIDController()
        self.flyPIDController.setFeedbackDevice(self.flyEncoder)

        self.flyPIDController.setP(SuperStrucConstants.kflyP)
        self.flyPIDController.setI(SuperStrucConstants.kflyI)
        self.flyPIDController.setD(SuperStrucConstants.kflyD)
        self.flyPIDController.setFF(SuperStrucConstants.kflyFF)

        # limit switches
        self.magazineSwitch = self.flyMotor.getForwardLimitSwitch(
            SparkMaxLimitSwitch.Type.kNormallyOpen
        )

        self.magazineSwitch.enableLimitSwitch(False)

        # Disable soft limits
        self.flyMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, False)
        self.flyMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, False)

        # Set dir
        # (joe added this, its bad)
        self.flyPIDController.setOutputRange(-1, 1)
        self.rotateMotor.setIdleMode(rev._rev.CANSparkBase.IdleMode.kBrake)

        # Burn flymotor configuration
        self.flyMotor.burnFlash()
        self.rotateMotor.burnFlash()

    def periodic(self) -> None:
        self.sd.putNumber("Thermals/rotate", self.rotateMotor.getMotorTemperature())
        self.sd.putNumber("Thermals/fly", self.flyMotor.getMotorTemperature())

    def setFlyWheelSpeed(self, speed):
        self.flyMotor.set(speed)

    def getAngle(self) -> float:
        """Return the current angle"""
        return self.rotateEncoder.getPosition()

    def isMagazineLoaded(self):
        """
        Returns true when the magazine is loaded with a note
        """

        return self.magazineSwitch.get()

    def rotateManual(self, speed):
        self.rotateMotor.set(speed)

    def setRotateAngle(self, angle: float):
        self.rotatePIDController.setReference(angle, CANSparkMax.ControlType.kPosition)

    def isReady(self):
        return self.flyEncoder.getVelocity() > 5000  # Magic

    def currentSpeed(self) -> float:
        return self.flyEncoder.getVelocity()

    def setIdleBrake(self):
        self.flyMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.flyMotor.burnFlash()

    def setIdleCoast(self):
        self.flyMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.flyMotor.burnFlash()

    def setFlyAngle(self, angle: float):
        self.flyPIDController.setReference(angle, CANSparkMax.ControlType.kPosition)

    def zeroFly(self):
        self.flyEncoder.setPosition(0)

    def getVelocity(self) -> float:
        return self.flyEncoder.getVelocity()

    def setFlyVelocity(self, velocity):
        self.flyPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity)

    # hello burflash my old friend
    def setIdleMode(self, mode):
        self.rotateMotor.setIdleMode(mode)
        self.rotateMotor.burnFlash()
