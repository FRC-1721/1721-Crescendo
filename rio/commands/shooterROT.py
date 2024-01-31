import commands2


class ShooterROT(commands2.Command):
    def __init__(self, angle, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem

        # requested speed
        self.angle = angle

    def initialize(self):
        self.subsystem.setRotateAngle(self.angle)

    def end(self, interrupted: bool):
        return True
