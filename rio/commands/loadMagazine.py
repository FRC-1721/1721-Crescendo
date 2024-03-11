import logging
import commands2

from subsystems.shooter import Shooter
from subsystems.intake import IntakeSubsystem


class LoadMagazine(commands2.Command):
    """
    Spin the flywheel slowly until note is loaded and ready 2 go
    """

    def __init__(self, _shooter: Shooter, _intake: IntakeSubsystem):
        super().__init__()

        # local subsystem instance
        self.shooter = _shooter
        self.intake = _intake

        # Command requirements
        self.addRequirements((self.shooter, self.intake))

    def initialize(self):
        logging.info("Spinning up...")
        self.shooter.setIdleBrake()

    def execute(self):
        self.intake.intake(-0.6)  # Slowly eject

        self.shooter.setFlyWheelSpeed(0.15)
        # self.shooter.setFlyVelocity(100)  # Slowly spin the flywheel
        print(self.shooter.getVelocity())

        if self.shooter.isMagazineLoaded():
            self.shooter.zeroFly()
            self.shooter.setFlyAngle(0.1)  # Four rotations forward

    def isFinished(self):
        return self.shooter.isMagazineLoaded()

    def end(self, interrupted: bool):
        self.intake.intake(0)
        # self.shooter.setFlyWheelSpeed(0)
        self.shooter.setIdleCoast()

        if not interrupted:
            logging.info(f"Done, magazine loaded")
        else:
            logging.warn("Load Magazine was inturrupted or cancled!")
