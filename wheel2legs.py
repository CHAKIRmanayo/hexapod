import motorcortex
import time
from math import *
from robot_control.motion_program import Waypoint, MotionProgram
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
import threading
import sys
    
class MCX:
    def __init__(self, hostname):
        self.hostname = hostname
        self.jointPosition = 'root/Control/jointManualPositions'
        self.pauseMode = 'root/Control/gotoPauseMode'
        self.manualMode = 'root/Control/gotoManualMode'
        self.actualPositionCmd = 'root/Control/actualJointPositionsFiltered'
        self.jointGeneratorList = ['root/Control/jointAutoSetpointGenerator01', 'root/Control/jointAutoSetpointGenerator02', 'root/Control/jointAutoSetpointGenerator03',
                                   'root/Control/jointAutoSetpointGenerator04', 'root/Control/jointAutoSetpointGenerator05', 'root/Control/jointAutoSetpointGenerator06',
                                   'root/Control/jointAutoSetpointGenerator07', 'root/Control/jointAutoSetpointGenerator08', 'root/Control/jointAutoSetpointGenerator09',
                                   'root/Control/jointAutoSetpointGenerator10', 'root/Control/jointAutoSetpointGenerator11', 'root/Control/jointAutoSetpointGenerator12',
                                   'root/Control/jointAutoSetpointGenerator13', 'root/Control/jointAutoSetpointGenerator14', 'root/Control/jointAutoSetpointGenerator15',
                                   'root/Control/jointAutoSetpointGenerator16', 'root/Control/jointAutoSetpointGenerator17', 'root/Control/jointAutoSetpointGenerator18',]
        
        ######Motor parameters######
        
        self.velocity = 1.0
        self.acceleration = 1.0
        self.jerk = 1.0
        
        ############################

        self.actualPosition = []
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
            
            self.setupParameterMotor(self.jointGeneratorList, self.acceleration, self.jerk, self.velocity)
            
            print('Complited motor')
            self.readCurrentPose()

        except RuntimeError as err:
            print(err)
            exit()

    def setupParameterMotor(self, path, maxAcc, maxJerk, maxVel):
        for i in range(len(path)):
            self.req.setParameter(path[i] + '/maxAcc', maxAcc)
            self.req.setParameter(path[i] + '/maxJerk', maxJerk)
            self.req.setParameter(path[i] + '/maxVel', maxVel)

    def setPauseMode(self, value):
        self.req.setParameter(self.pauseMode, value).get()

    def setManualMode(self, value):
        self.req.setParameter(self.manualMode, value).get()

    def setMotorPosition(self, value):
        for i in range(len(value)):
            self.req.setParameter(self.jointGeneratorList[i] + '/input', value[i]).get()

    def setMotorPositionByIndex(self, index, value):
        self.req.setParameter(self.jointGeneratorList[index] + '/input', value).get()

    def readCurrentPose(self):
        def callback():
            while True:
                get_param_reply_msg = self.req.getParameter(
                    self.actualPositionCmd).get()
                self.actualPosition = get_param_reply_msg.value
                time.sleep(0.001)
        
        threading.Thread(target=callback, daemon=True).start()

    def getActualPosition(self):
        return self.actualPosition
    
    def move_motor_to_points(self, target_positions, motorIdx, axis):
        index = 0
        while index < len(target_positions):
            boolTarget = []
            target_position = target_positions[index]
            for i in range(len(target_position)):
                boolTarget.append(False)
                for j in range(len(motorIdx)):
                    self.setMotorPositionByIndex(motorIdx[j][i], target_position[i] * axis[j][i])
                    print(f"Двигаем мотор {motorIdx[j][i]} к позиции: {target_position[i] * axis[j][i]}")

            while True:
                actual_position = self.getActualPosition()
                for i in range(len(target_position)):
                    for j in range(len(motorIdx)):
                        if abs(actual_position[motorIdx[j][i]] - (target_position[i] * axis[j][i])) < 0.001:
                            boolTarget[i] = True

                if all(boolTarget):
                    print(f"Моторы достигли целевой позиции")
                    time.sleep(0.01)
                    break

                time.sleep(0.01)  

            index += 1

    
def moveToTarget(mcx, target):
    targetPositionArray = []
    for i in range(18):
        targetPositionArray.append(target)

    mcx.setMotorPosition(targetPositionArray) #move center

def moveToStart(mcx, targetPositionArray):
    print("Move to start")
    mcx.setMotorPosition(targetPositionArray)
    while True:
        actualMotoPosition = mcx.getActualPosition()
    
        if all(x > -0.001 for x in actualMotoPosition) and all(x < 0.001 for x in actualMotoPosition):
            print("Now is start")
            break

def main():
    start = False
    try:
        if sys.argv[1]:
            if sys.argv[1].lower() == 'true' or sys.argv[1].lower() == 'false':
                if sys.argv[1].lower() == 'true':
                    wheel2legs = True
                else:
                    wheel2legs = False
                start = True
            else:
                print("Write argument as 'true' or 'false'")
    except:
        print("Write argument as 'true' or 'false'\nExample: python3 wheel2legs.py false")

    if start:
        mcx = MCX('192.168.2.100')
        time.sleep(2)

        pt_path_BR = [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [3.925, 1.255, 0.9], [3.925, 1.542, 1.876]]
        pt_path_BL = [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [-3.925, 1.255, 0.9], [-3.925, 1.542, 1.876]]
        pt_path_FL = [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [3.925, 1.255, 0.9], [3.925, 1.542, 1.876]]
        pt_path_FR = [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 0.9], [0.0, 1.255, 0.9], [-3.925, 1.255, 0.9], [-3.925, 1.542, 1.876]]
        pt_path_M = [[0.0, 0.0, 0.0], [0.0, -0.316, 0.313], [0.0, -0.316, 1.895], [0.0, 0.096, 1.895], [-2.356, 0.096, 1.895]]

        pt_path_reverse_BR = list(reversed(pt_path_BR))
        pt_path_reverse_BL = list(reversed(pt_path_BL))
        pt_path_reverse_FL = list(reversed(pt_path_FL))
        pt_path_reverse_FR = list(reversed(pt_path_FR))
        pt_path_reverse_BR = list(reversed(pt_path_BR))
        pt_path_reverse_M = list(reversed(pt_path_M))
        

        mcx.setPauseMode(0)
        mcx.setManualMode(0)

        time.sleep(1)

        if not(wheel2legs):
            

            mcx.move_motor_to_points(pt_path_FL, [[3, 4, 5], [12, 13, 14]], [[1, 1, 1], [1, 1, 1]])
            mcx.move_motor_to_points(pt_path_FR, [[6, 7, 8], [15, 16, 17]], [[1, 1, 1], [1, 1, 1]])
            mcx.move_motor_to_points(pt_path_M, [[0, 1, 2], [9, 10, 11]], [[1, 1, 1], [1, 1, 1]])
            
        else:
            mcx.move_motor_to_points(pt_path_reverse_M, [[0, 1, 2], [9, 10, 11]], [[1, 1, 1], [1, 1, 1]])
            mcx.move_motor_to_points(pt_path_reverse_FR, [[6, 7, 8], [15, 16, 17]], [[1, 1, 1], [1, 1, 1]])
            mcx.move_motor_to_points(pt_path_reverse_FL, [[3, 4, 5], [12, 13, 14]], [[1, 1, 1], [1, 1, 1]])

            

if __name__ == "__main__":
    main()