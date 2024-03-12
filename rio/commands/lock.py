import commands2

from constants import ClimberConstants


class Lock(commands2.Command):
    def __init__(self, subsystem, locktype):
        """
        rotates the climber
        subsystem: the intake
        locktype: is it open or closed
        """
        super().__init__()

        # local instance of subsystem
        self.climberSubsystem = subsystem

        if locktype == "lock":
            self.locktype = ClimberConstants.kServoLock

        else:
            self.locktype = ClimberConstants.kservoOpen

    def initialize(self):
        self.climberSubsystem.setServoAngle(self.locktype)

    def end(self, interrupted: bool):
        return True
