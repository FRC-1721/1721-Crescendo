import commands2

from constants import ClimberConstants

from subsystems.climber import Climber


class Lock(commands2.Command):
    def __init__(self, _climber: Climber, locktype):
        """
        rotates the climber
        subsystem: the intake
        locktype: is it open or closed
        """
        super().__init__()

        # local instance of subsystem
        self.climber = _climber

        if locktype == "lock":
            self.locktype = ClimberConstants.kServoLock
        else:
            self.locktype = ClimberConstants.kservoOpen

    def initialize(self):
        self.climber.setServoAngle(self.locktype)

    def end(self, interrupted: bool):
        return True
