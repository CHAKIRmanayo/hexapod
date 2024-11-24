import motorcortex
import time
from math import *
from robot_control.motion_program import Waypoint, MotionProgram
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import numpy as np


class MCX:
    def __init__(self, hostname):
        self.hostname = hostname
        self.jointPosition = 'root/Control/jointManualPositions'
        self.pauseMode = 'root/Control/gotoPauseMode'
        self.manualMode = 'root/Control/gotoManualMode'
        self.jointManualVelocity = 'root/Control/gotoPauseMode/jointManualVelocity'

        for i in range(1, 19):
            setattr(self, f'jointGenerator{i:02}', f'root/Control/jointAutoSetpointGenerator{i:02}')


        self.connect()

    def connect(self):
        parameter_tree = motorcortex.ParameterTree()
        self.messageTypes = motorcortex.MessageTypes()
        try:
            self.req, self.sub = motorcortex.connect("wss://" + self.hostname + ":5568:5567",
                                                self.messageTypes, parameter_tree,
                                                certificate="mcx.cert.pem", timeout_ms=1000,
                                                login="admin", password="vectioneer")
                            
            self.robot = RobotCommand(self.req, self.messageTypes)
            
            if self.robot.engage():
                print('Robot is at Engage')
            else:
                raise Exception('Failed to set robot to Engage')
            
            for i in range(1, 19):
                generator = getattr(self, f'jointGenerator{i:02}')
                self.setupParameterMotor(generator, 1, 1, 10)


        except RuntimeError as err:
            print(err)
            exit()

    def setupParameterMotor(self, path, maxAcc, maxJerk, maxVel):
        self.req.setParameter(path + '/maxAcc', maxAcc)
        self.req.setParameter(path + '/maxJerk', maxJerk)
        self.req.setParameter(path + '/maxVel', maxVel)

    def setPauseMode(self, value):
        self.req.setParameter(self.pauseMode, value).get()

    def setManualMode(self, value):
        self.req.setParameter(self.manualMode, value).get()

    def setVelocity(self, value):
        self.req.setParameter(self.jointManualVelocity, value).get()

    def setMotorsPosition(self, values):
        for i, val in enumerate(values, start=1):
            generator_input = getattr(self, f'jointGenerator{i:02}') + '/input'
            self.req.setParameter(generator_input, val).get()
    
    def setDxlPosition(self, value):
        self.req.setParameter(self.dynamixelPosition, value[0]).get()
        

    def getAndSetFromDeskPosition(self):
        paramFromDynamixel = self.req.getParameter(self.dynamixelPosition)
        self.setDxlPosition([paramFromDynamixel.get().value[0]])

    def setDxlTorueEnable(self, value):
        pass


startValues = np.zeros(18)
finishValues = [0.8]*18

def main():
    mcx = MCX('192.168.2.100')
    mcx.setPauseMode(0) 
    mcx.setManualMode(0)
    for t in np.arange(0, np.pi, 0.01):
        # a = np.sin(t*0.01+1)
        # mcx.setMotorsPosition([a]*18)
        time.sleep(0.1)
        mcx.setPauseMode(0) 
        mcx.setManualMode(0)
    mcx.req.close()
    mcx.sub.close()

if __name__ == "__main__":
    main()
    