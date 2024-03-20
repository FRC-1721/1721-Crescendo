import typing
import logging
import commands2

from constants import ClimberConstants

from subsystems.climber import Climber
from subsystems.shooter import Shooter


class Spool(commands2.Command):
    def __init__(
        self,
        _speed: typing.Callable[
            [],
            float,
        ],
        _climber: Climber,
        _shooter: Shooter,
    ):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climber = _climber
        self.shooter = _shooter

        # local var of speed
        self.speed = _speed

        # Command requires
        self.addRequirements((self.climber, self.shooter))

        self.climber.setServoAngle(ClimberConstants.kServoLock)

    def execute(self):
        logging.debug(f"Climb is {self.speed}")

        if self.speed == 0 or self.speed >= 0.5:
            # Positive numbers imply we're pulling DOWN
            self.shooter.currlimit(1)
            self.climber.setServoAngle(ClimberConstants.kServoLock)
            # self.shooter.rotateManual(
            #    -self.speed * ClimberConstants.kClimberShooterForward
            # )
        else:
            # Negative numbers imply we're relaxing
            self.shooter.currlimit(2)
            self.climber.setServoAngle(ClimberConstants.kservoOpen)
            # self.shooter.rotateManual(
            #    -self.speed * ClimberConstants.kClimberShooterBackward
            # )

        self.climber.setClimberMotorSpeed(self.speed)

    def end(self, interrupted: bool):
        self.climber.setServoAngle(ClimberConstants.kServoLock)
        self.shooter.currlimit(35)
        self.climber.setClimberMotorSpeed(0)
        self.shooter.rotateManual(0)
