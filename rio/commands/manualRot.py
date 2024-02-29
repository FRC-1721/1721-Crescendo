import commands2


class manualROT(commands2.Command):
    def __init__(self, speed, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem
        self.speed = speed
        # requested speed

        self.addRequirements(self.subsystem)

    def execute(self):
        self.subsystem.rotateManual(self.speed)

    def end(self, interrupted: bool):
        self.subsystem.rotateManual(0)
        return True
