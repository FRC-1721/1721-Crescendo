import commands2
import logging

class ShooterROT(commands2.Command):
    def __init__(self, angle, subsystem):
        super().__init__()

        # local subsystem instance
        self.subsystem = subsystem
        
        # requested speed
        self.angle = angle

    def initialize(self):
        logging.info(f"Moving intake to {self.angle}")

    def execute(self):
        self.subsystem.setRotateAngle(self.angle)
        
        logging.info(
            f"Still moving... {self.subsystem.getAngle()} to {self.angle}"
        )
    def end(self, interrupted: bool):
        return True
