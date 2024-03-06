# Used to bypass bugs in the gyro init during sim
class DummyGyro:
    def __init__(self):
        pass

    def getYaw(self) -> float:
        return 0.0

    def configMountPose(self, x, y, z):
        pass
