import commands2

from constants import ClimberConstants


class Climb(commands2.Command):
    def __init__(self, subsystem, speed):
        """
        rotates the climber
        """
        super().__init__()

        # local instance of subsystem
        self.climberSubsystem = subsystem

        # local var of speed
        self.speed = speed

    def initialize(self):
        self.climberSubsystem.setServoAngle(ClimberConstants.kservoOpen)
        commands2.waitcommand(1)  # to not grind on the servo lock
        self.climberSubsystem.setClimberMotorSpeed(self.speed)

    def end(self, interrupted: bool):
        self.climberSubsystem.setClimberMotorSpeed(0)
        commands2.waitcommand(1)
        self.climberSubsystem.setServoAngle(ClimberConstants.kservolock)
