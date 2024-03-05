import commands2

from subsystems.shooter import Shooter


class manualROT(commands2.Command):
    def __init__(self, speed, _shooter: Shooter):
        super().__init__()

        # local subsystem instance
        self.subsystem = Shooter

        # requested speed
        self.speed = speed

        # Command requirements
        self.addRequirements(self.subsystem)

    def execute(self):
        self.subsystem.rotateManual(self.speed)

    def end(self, interrupted: bool):
        self.subsystem.rotateManual(0)
        return True
