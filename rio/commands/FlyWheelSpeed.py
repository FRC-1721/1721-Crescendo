import commands2


class FlyWheelSpeed(commands2.Command):
    def __init__(self, speed, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem

        # requested speed
        self.speed = speed

    def initialize(self):
        self.subsystem.setFlyWheelSpeed(self.speed)

    def end(self, interrupted: bool):
        self.subsystem.setFlyWheelSpeed(0)
        return True
