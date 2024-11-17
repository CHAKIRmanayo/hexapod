import motorcortex
import time
from math import *
from robot_control.motion_program import Waypoint, MotionProgram
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import _thread

class MCX:
    def __init__(self, hostname):
        self.hostname = hostname
        self.jointPosition = 'root/Control/jointManualPositions'
        self.pauseMode = 'root/Control/gotoPauseMode'
        self.manualMode = 'root/Control/gotoManualMode'
        self.jointManualVelocity = 'root/Control/gotoPauseMode/jointManualVelocity'
        self.jointGenerator01 = 'root/Control/jointAutoSetpointGenerator01'
        self.jointGenerator02 = 'root/Control/jointAutoSetpointGenerator02'
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
                

                self.subscription = self.sub.subscribe(['root/ManipulatorControl/fkActualToolCoords/toolCoordinates'], 'group1', 5)
                self.is_subscribed = self.subscription.get()

                print("Subscribtion", self.is_subscribed)
                
                _thread.start_new_thread(self.printParam, ("Thread-1", 0.2, ))

            except RuntimeError as err:
                print(err)
                exit()

    def printParam(self, threadName, delay):
 
        while True:
            # print("then")
            params = self.subscription.read()
            print(params[0].value)
            time.sleep(delay)

def main():
    mcx = MCX('192.168.2.100')

if __name__ == "__main__":
    main()