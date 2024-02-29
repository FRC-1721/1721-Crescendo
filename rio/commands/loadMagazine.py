import logging
import commands2

from subsystems.shooter import Shooter


class LoadMagazine(commands2.Command):
    """
    Spin the flywheel slowly until note is loaded and ready 2 go
    """

    def __init__(self, speed: float, _shooter: Shooter):
        super().__init__()

        # local subsystem instance
        self.shooter = _shooter

        # requested speed
        self.speed = speed

    def initialize(self):
        logging.info("Spinning up...")
        self.shooter.setIdleBrake()

    def execute(self):
        self.shooter.setFlyWheelSpeed(self.speed)
        if self.shooter.isMagazineLoaded():
            self.shooter.zeroFly()
            self.shooter.setFlyAngle(0)

    def isFinished(self):
        return self.shooter.isMagazineLoaded()

    def end(self, interrupted: bool):
        self.shooter.setFlyWheelSpeed(0)
        self.shooter.setIdleCoast()
        logging.info(f"Done, magazine loaded")
