import commands2

from ntcore import NetworkTableInstance
from rev import CANSparkMax, CANSparkLowLevel
from constants.constants import getConstants


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        """
        creates a intake subsystem
        """
        super().__init__()

        # dashboard
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")

        # hardware consts
        constants = getConstants("robot_hardware")
        self.intakeConsts = constants["intake"]

        # lifting motor
        self.liftMotor = CANSparkMax(
            self.intakeConsts["LiftPort"],
            CANSparkLowLevel.MotorType.kBrushless,
        )

        # intaking motor
        self.intakeMotor = CANSparkMax(
            self.intakeConsts["IntakePort"],
            CANSparkLowLevel.MotorType.kBrushless,
        )

        # setting iverted
        self.liftMotor.setInverted(self.intakeConsts["LiftInverted"])
        self.intakeMotor.setInverted(self.intakeConsts["IntakeInverted"])

    def periodic(self) -> None:
        self.sd.putNumber("Thermals/Intake", self.intakeMotor.getMotorTemperature())
        self.sd.putNumber("Thermals/Lift", self.liftMotor.getMotorTemperature())

    def intake(self, speed):
        self.intakeMotor.set(speed)

    def lift(self, speed):
        self.liftMotor.set(speed)

    def liftCurrentLimit(self, current):
        self.liftMotor.setSmartCurrentLimit(current)

    def intakeCurrentLimit(self, current):
        self.intakeMotor.setSmartCurrentLimit(current)
