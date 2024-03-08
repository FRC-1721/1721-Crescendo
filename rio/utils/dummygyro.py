# Used to bypass bugs in the gyro init during sim
class DummyGyro:
    def __init__(self):
        pass

    def getAngle(self) -> float:
        return 0.0

    def getBoardYawAxis(self) -> None:
        return None

    def setAngleAdjustment(self, num):
        pass
