import commands2


class spinGuideMotor(commands2.Command):
    def __init__(self, subsystem, speed):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem

        self.speed = speed

    def initialize(self):
        self.subsystem.setGuidingMotorSpeed(self.speed)

    def end(self, interrupted: bool):
        self.subsystem.guiding(0)
        return True
