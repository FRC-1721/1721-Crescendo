import commands2

from constants import ClimberConstants

from subsystems.climber import Climber


class Spool(commands2.Command):
    def __init__(
        self,
        _speed,
        _climber: Climber,
    ):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climber = _climber

        # local var of speed
        self.speed = _speed

        # Command requires
        self.addRequirements((self.climber))

    def initialize(self):
        self.climber.setServoAngle(ClimberConstants.kservoOpen)

    def execute(self):
        self.climber.setClimberMotorSpeed(self.speed)

    def end(self, interrupted: bool):
        self.climber.setClimberMotorSpeed(0)
