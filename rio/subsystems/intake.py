import commands2
import logging

from ntcore import NetworkTableInstance
from rev import CANSparkMax, CANSparkLowLevel, SparkAbsoluteEncoder, SparkMaxLimitSwitch
from constants import IntakeConstants



class IntakeSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        """
        creates a intake subsystem
        """
        super().__init__()

        # dashboard
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # lifting motor
        self.liftMotor = CANSparkMax(
            IntakeConstants.kliftCanId,
            CANSparkLowLevel.MotorType.kBrushless,
        )

        # intaking motor
        self.intakeMotor = CANSparkMax(
            IntakeConstants.kIntakeCanId,
            CANSparkLowLevel.MotorType.kBrushless,
        )

        self.intakeMotor.restoreFactoryDefaults()

        self.intakeMotor.setInverted(True)

        # encoders
        self.liftEncoder = self.liftMotor.getAbsoluteEncoder(
            SparkAbsoluteEncoder.Type.kDutyCycle
        )
        self.liftEncoder.setPositionConversionFactor(IntakeConstants.kLiftConversion)

        self.intakeEncoder = self.intakeMotor.getEncoder()

        # setting inverted
        self.liftMotor.setInverted(IntakeConstants.kLiftInversion)
        # self.intakeMotor.setInverted(IntakeConstants.kIntakeInversion)

        # pids
        self.liftPID = self.liftMotor.getPIDController()
        self.liftPID.setP(IntakeConstants.kLiftP)
        self.liftPID.setI(IntakeConstants.kLiftI)
        self.liftPID.setD(IntakeConstants.kLiftD)
        self.liftPID.setFF(IntakeConstants.kLiftFF)

        # limit switches
        self.rightLimitSwitch = self.intakeMotor.getForwardLimitSwitch(
            SparkMaxLimitSwitch.Type.kNormallyOpen
        )

        self.rightLimitSwitch.enableLimitSwitch(True)

        self.intakeMotor.burnFlash()

    def periodic(self) -> None:
        self.sd.putNumber("Thermals/Intake", self.intakeMotor.getMotorTemperature())
        self.sd.putNumber("Intake/IntakeAngle", self.intakeEncoder.getPosition())
        self.sd.putNumber("Thermals/Lift", self.liftMotor.getMotorTemperature())

    def intake(self, speed):
        if speed > 0:
            if not self.rightLimitSwitch.get():
                self.intakeMotor.set(speed)
        else:
            self.intakeMotor.set(speed)

    def manualLift(self, speed):
        self.liftMotor.set(speed)

    def lift(self, angle: float):
        self.liftPID.setReference(angle, CANSparkMax.ControlType.kPosition)

    def liftCurrentLimit(self, current):
        self.liftMotor.setSmartCurrentLimit(current)

    def intakeCurrentLimit(self, current):
        self.intakeMotor.setSmartCurrentLimit(current)
