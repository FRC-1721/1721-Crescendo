from rev import CANSparkMax, CANSparkMaxLowLevel
from constants.constants import getConstants
from wpilib import DutyCycleEncoder


class SwerveModule:
    def __init__(
        self,
        driveChannel,
        turnChannel,
        driveEncoder,
        turnEncoder,
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
        self.drivePID = constants["DrivePIDValues"]
        self.turnPID = constants["TurnPIDValues"]

        # motors
        self.driveMotor = CANSparkMax(
            driveChannel, CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.turnMotor = CANSparkMax(
            turnChannel, CANSparkMaxLowLevel.MotorType.kBrushless
        )

        # encoders
        self.driveEncoder = self.driveMotor.getEncoder()
        self.turnEncoder = self.turnMotor.getEncoder()

        # PID
        self.drivePIDController = self.driveMotor.getPIDController()
        self.turnPIDController = self.turnMotor.getPIDController()

        self.drivePIDController.setP(self.drivePID["P"])
        self.drivePIDController.setI(self.drivePID["I"])
        self.drivePIDController.setD(self.drivePID["D"])
        self.drivePIDController.setFF(self.drivePID["FF"])

        self.turnPIDController.setP(self.turnPID["P"])
        self.turnPIDController.setI(self.turnPID["I"])
        self.turnPIDController.setD(self.turnPID["D"])
        self.turnPIDController.setFF(self.turnPID["FF"])

    # Copied from rev, be skeptical
    def find_zero(self):
        """
        remember how much this sucked
        last year, zero's serve maybe
        """
        # this may be the wrong motor
        self.turnMotor.setPosition(0)
