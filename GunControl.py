import motorcortex
import time
from math import *
from robot_control.motion_program import Waypoint, MotionProgram
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
from Dynamixel import XM430, Control

class MCX:
    def __init__(self, hostname):
        self.hostname = hostname
        self.jointPosition = 'root/Control/jointManualPositions'
        self.pauseMode = 'root/Control/gotoPauseMode'
        self.manualMode = 'root/Control/gotoManualMode'
        self.jointManualVelocity = 'root/Control/gotoPauseMode/jointManualVelocity'
        self.jointGenerator01 = 'root/Control/jointAutoSetpointGenerator01'
        self.jointGenerator02 = 'root/Control/jointAutoSetpointGenerator02'

        self.dynamixelPosition = 'root/UserParameters/dynamixel'

        self.dxlList = [XM430(1)]
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
            
            self.setupParameterMotor(self.jointGenerator01, 0.2, 0.2, 0.2)
            self.setupParameterMotor(self.jointGenerator02, 0.2, 0.2, 0.2)

        except RuntimeError as err:
            print(err)
            self.dxlControl.closePort()
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

    def setMotorPosition(self, value):
        self.req.setParameter(self.jointGenerator01 + '/input', value[0]).get()
        self.req.setParameter(self.jointGenerator02 + '/input', value[1]).get()
    
    def setDxlPosition(self, value):
        self.req.setParameter(self.dynamixelPosition, value[0]).get()
        self.dxlControl.setGoalPosition(self.dxlList, value)

    def getAndSetFromDeskPosition(self):
        paramFromDynamixel = self.req.getParameter(self.dynamixelPosition)
        self.setDxlPosition([paramFromDynamixel.get().value[0]])

    def setDxlTorueEnable(self, value):
        self.dxlControl.setTorqueEnable(self.dxlList, [value])

def main():
    mcx = MCX('192.168.2.100')

    mcx.setPauseMode(0)
    mcx.setManualMode(0)

    mcx.setMotorPosition([0.0, 0.0]) #move center
    time.sleep(5)

    mcx.setMotorPosition([0.25, 0.25]) #move right and up gun
    time.sleep(5)

    # mcx.setMotorPosition([-0.25, 0.25]) #move left and up gun
    # time.sleep(5)

    # mcx.setMotorPosition([0.0, 0.0]) #move center
    # time.sleep(5)

    # mcx.setDxlTorueEnable(1)
    # mcx.setDxlPosition([3333]) #move to center
    # time.sleep(5)

if __name__ == "__main__":
    main()