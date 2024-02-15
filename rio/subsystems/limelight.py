
from ntcore import NetworkTableInstance

from wpilib import RobotBase

import commands2
import json


class limeLightCommands(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        # Configure networktables
        self.nt = NetworkTableInstance.getDefault()
        self.sd = self.nt.getTable("SmartDashboard")
        self.ll = self.nt.getTable("limelight-jack")
        self.fieldSize = [16, 8]

    def setPipeline(self, PipeLine:int) -> None:
        self.ll.getEntry("pipeline").setDouble(PipeLine)

    def periodic(self) -> None:
        pass

    def getPoseInField(self) -> None:
        if self.nt.getTable("FMSinfo").getEntry("IsRedAlliance") is True:
            posinfield = self.ll.getEntry("Botpose_wpired").getDoubleArrayTopic(0)
        else:
            posinfield = self.ll.getEntry("Botpose_wpiblue").getDoubleArrayTopic(0)

        self.botPos = [posinfield[0],posinfield[1],posinfield[5]]

    def sendToPos(self,PosX,PosY,RotZ, driver) -> None:
        """
        compares the inputted location to the robots current position and sends it roughly in the direction of the desired position.

        PosX: the robot's X position starting at 0 from freindly side and extending to 16 down the long side of the field towards the opposing alliances side
        PosY: the robot's Y position starting at 0 from the right side of the field relative to the drivers point of view and extending to 8 towards the left side of the field.
        RotZ: Further testing required regarding the specifics of the robot's Z rotation.

        NOTE: The data from the networktables is sent like this [TX,TY,TZ,RX,RY,RZ,?] we only care about TX,TY, and RZ
        """

        if PosX > 16 or PosY > 8:
            print("Desired location out of field perimeters! Let's try to think logically next time bud!")
            return
        
        driveX = 0
        driveY = 0
        RotZ = 0
        correctPos = True

        if (self.botPos[0] < PosX - 0.05) or (self.botPos[0] > PosX + 0.05): #error tolernaces are placeholder values
            driveX = (PosX - self.botPos[0])/10 #drive speed is placeholder & guess of what we might want to do in order to get fast results & high accuracy using dampening will need to be tweaked & adjusted & such things
            correctPos = False

        if (self.botPos[1] < PosY - 0.05) or (self.botPos[1] > PosY + 0.05): #error tolernaces are placeholder values
            driveY = (PosY - self.botPos[1])/10 #drive speed is placeholder & guess of what we might want to do in order to get fast results & high accuracy using dampening will need to be tweaked & adjusted & such things
            correctPos = False

        if (self.botPos[2] < RotZ - 0.5) or (self.botPos[2] > RotZ + 0.5): #error tolernaces are placeholder values
            driveZ = (RotZ - self.botPos[2])/10 #drive speed is placeholder & guess of what we might want to do in order to get fast results & high accuracy using dampening will need to be tweaked & adjusted & such things
            correctPos = False

        #NOTE commented out print commands to use while testing

        #print(self.botPos[0])
        #print(self.botPos[1])                                               ROBOT POSITIONS
        #print(self.botPos[2])
                
        #print(driveX)
        #print(driveY)                                                        DRIVE VALUES
        #print(RotZ)
            
        #print(correctPos)                                                                                 IS THE ROBOT IN THE RIGHT PLACE
            
        driver.drive(driveX,driveY,driveZ,False,True)       