# Used to bypass bugs in the gyro init during sim
class DummyGyro:
    def __init__(self):
        pass

    def getAngle(self) -> float:
        return 0.0
