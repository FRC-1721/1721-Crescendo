from ntcore import NetworkTableInstance

from subsystems.drivesubsystem import DriveSubsystem

from wpilib import RobotBase

import math
import logging
import commands2
import json


class limeLightCommands(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # Configure networktables
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")
        self.ll = self.nt.getTable("limelight-jack")

        # field size in goofy limelight units
        self.fieldSize = [16, 8]

        # self.xFifo = array([0] * 10)
        # self.yFifo = array([0] * 10)

    def setPipeline(self, PipeLine: int) -> None:
        self.ll.getEntry("pipeline").setDouble(PipeLine)

    def periodic(self) -> None:
        pass

    def getPoseInField(self) -> None:
        if self.nt.getTable("FMSinfo").getEntry("IsRedAlliance") is True:
            posinfield = self.ll.getEntry("Botpose_wpired").getDoubleArrayTopic(0)
        else:
            posinfield = self.ll.getEntry("Botpose_wpiblue").getDoubleArrayTopic(0)

        self.botPos = [posinfield[0], posinfield[1], posinfield[5]]

    def sendToPos(self, PosX, PosY, RotZ, driver) -> None:
        """
        compares the inputted location to the robots current position and sends it roughly in the direction of the desired position.

        PosX: the robot's X position starting at 0 from freindly side and extending to 16 down the long side of the field towards the opposing alliances side
        PosY: the robot's Y position starting at 0 from the right side of the field relative to the drivers point of view and extending to 8 towards the left side of the field.
        RotZ: Further testing required regarding the specifics of the robot's Z rotation.

        NOTE: The data from the networktables is sent like this [TX,TY,TZ,RX,RY,RZ,?] we only care about TX,TY, and RZ
        """

        if PosX > 16 or PosY > 8:
            logging.warn(
                "Specified location out of field perimeters! Let's try to think logically next time bud!"
            )
            return

        driveX = 0
        driveY = 0
        RotZ = 0
        correctPos = True

        if (self.botPos[0] < PosX - 0.05) or (
            self.botPos[0] > PosX + 0.05
        ):  # error tolernaces are placeholder values
            driveX = (
                PosX - self.botPos[0]
            ) / 10  # drive speed is placeholder & guess of what we might want to do in order to get fast results & high accuracy using dampening will need to be tweaked & adjusted & such things
            correctPos = False

        if (self.botPos[1] < PosY - 0.05) or (
            self.botPos[1] > PosY + 0.05
        ):  # error tolernaces are placeholder values
            driveY = (
                PosY - self.botPos[1]
            ) / 10  # drive speed is placeholder & guess of what we might want to do in order to get fast results & high accuracy using dampening will need to be tweaked & adjusted & such things
            correctPos = False

        if (self.botPos[2] < RotZ - 0.5) or (
            self.botPos[2] > RotZ + 0.5
        ):  # error tolernaces are placeholder values
            driveZ = (
                RotZ - self.botPos[2]
            ) / 10  # drive speed is placeholder & guess of what we might want to do in order to get fast results & high accuracy using dampening will need to be tweaked & adjusted & such things
            correctPos = False
        driver.drive(driveX, driveY, driveZ, True, True)

    def findObj(self) -> bool:
        # tries to recieve the objects distance from the robot via NetworkTables
        posX = self.ll.getEntry("tx").getDouble(0)
        posY = self.ll.getEntry("ty").getDouble(0)

        if (posX == 0) and (posY == 0):
            return False
        else:
            # append(self.xFifo, posX)  # Append newly collected Pos to array of data to
            # append(self.yFifo, posY)  # collect an average distance

            # self.xFifo = delete(self.xFifo, 0)              # Deletes old Pos variables that we don't really need
            # self.yFifo = delete(self.yFifo, 0)

            # TODO: change pos variables to numpy array variables
            self.distX = posX
            self.distY = posY
            return True

    def goToObj(self, driver: DriveSubsystem) -> None:
        driveX = 0

        if self.distY > 0:
            driveY = -1 * (math.pow(0.25, 0.125 * self.distY + 1)) + 0.25
        else:
            driveY = 0

        if (self.distX < -0.1) or (self.distX > 0.1):
            driveX = -0.02 * self.distX
        else:
            driveX = 0
        print(driveY)
        driver.drive(driveY, driveX, 0, False, True)

    def isAtOBJ(self) -> bool:
        if (-0.1 < self.distY < 0.25) and (-0.25 < self.distX < 0.25):
            return True
        return False
