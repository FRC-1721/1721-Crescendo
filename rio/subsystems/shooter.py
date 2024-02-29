import wpilib
from ntcore import NetworkTableInstance
from commands2 import Subsystem
from rev import (
    CANSparkLowLevel,
    CANSparkMax,
    SparkAbsoluteEncoder,
    SparkMaxLimitSwitch,
    CANSparkBase
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

        self.rotateEncoder.setPositionConversionFactor(SuperStrucConstants.kRotConversion)

        # PID values
        self.rotatePIDController = self.rotateMotor.getPIDController()
        self.rotatePIDController.setFeedbackDevice(self.rotateEncoder)

        self.rotatePIDController.setP(SuperStrucConstants.kP)
        self.rotatePIDController.setI(SuperStrucConstants.kI)
        self.rotatePIDController.setD(SuperStrucConstants.kD)
        self.rotatePIDController.setFF(SuperStrucConstants.kFF)

        # limit switches
        self.limtSwitch = self.flyMotor.getForwardLimitSwitch(
            SparkMaxLimitSwitch.Type.kNormallyOpen
        )

        self.limtSwitch.enableLimitSwitch(True)

        self.flyMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward,False)
        self.flyMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse,False)
        # these are defaults

        self.flyMotor.burnFlash()

    def periodic(self) -> None:
        self.sd.putNumber("Thermals/rotate", self.rotateMotor.getMotorTemperature())
        self.sd.putNumber("Thermals/fly", self.flyMotor.getMotorTemperature())
        print(self.rotateEncoder.getPosition())

    def setFlyWheelSpeed(self, speed):
        self.flyMotor.set(speed)
    
    def getAngle(self) -> float:
        """Return the current angle"""
        return self.rotateEncoder.getPosition()
    
    def switchPress(self):
        return self.limtSwitch.get()

    def rotateManual(self, speed):
        self.rotateMotor.set(speed)

    def setRotateAngle(self, angle: float):
        self.rotatePIDController.setReference(angle, CANSparkMax.ControlType.kPosition)

    def isReady(self):
        return self.flyEncoder.getVelocity() > 5500  # Magic

    def setIdleBrake(self):
        self.flyMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        self.flyMotor.burnFlash()

    def setIdleCoast(self):
        self.flyMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        self.flyMotor.burnFlash()
