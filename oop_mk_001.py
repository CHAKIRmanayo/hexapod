import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import time


def rotationZ (theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
        ])


def rotationX (theta):
    return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])


class Meha:
    def __init__(self, position, basis):
        self.position= position
        self.basis = basis
        self.legs = {}

    def changePosition(self, ds):
        self.position += ds
    
    def changeBasis(fi):
        pass
    
    def addLeg(self, key, pos, angle):
        self.legs[key] = LEG(pos, angle)

    def getStatus(self):
        print('LEGS')
        offs = 10*' '
        for legId in self.legs:
            print(f'\n{30*'-'}')
            status = self.legs[legId].getStatus()
            for var in status:
                print(f'\n{str.upper(legId)} {var}:')
                for key in status[var]:
                    print(f'{offs}{key}: {status[var][key]}')
    


    
def vec3(x,y,z):
    return np.array([x,y,z])

class LEG:
    def __init__(self, pos, angle):
        self.J0 = pos
        self.BaseT10 = rotationZ(angle) @ rotationX(np.pi)
        self.BaseT21 = rotationX(-np.pi/2) @ rotationZ(-np.pi/2)
        self.BaseT32 = np.eye(3)
        # print(np.round(np.linalg.inv(self.BaseT10),3),'\n')
        # print(np.round(np.linalg.inv(self.BaseT21),3),'\n')
        # print(np.round(np.linalg.inv(self.BaseT32),3),'\n')
        self.s1 = vec3(0,0,1)
        self.s2 = vec3(3,0,0)
        self.s3 = vec3(3,0,0)
        self.rotateAllSustavs(0, 0, 0)
        self.calculate_Joints_Relative_Meha()
    
    def rotateAllSustavs(self, theta1, theta2, theta3):
        self.A1 = theta1
        self.A2 = theta2
        self.A3 = theta3
        self.T10 = rotationZ(theta1) # self.BaseT10 @ rotationZ(theta1)
        self.T21 = rotationZ(theta2) 
        self.T32 = rotationZ(theta3)
        # self.T10 = self.BaseT10 @ rotationZ(theta1)
        # self.T21 = self.BaseT21 @ rotationZ(theta2)
        # self.T32 = self.BaseT32 @ rotationZ(theta3)

    def calculate_Joints_Relative_Meha(self):
        R1 = self.BaseT10 @ self.T10
        R2 = self.BaseT21 @ R1 @ self.T21
        R3 = self.BaseT32 @ R2 @ self.T32
        self.J1 = self.J0 + R1 @ self.s1
        self.J2 = self.J1 + R2 @ self.s2
        self.J3 = self.J2 + R3 @ self.s3

        # print(np.round(np.linalg.inv(R1),3),'\n')
        # print(np.round(np.linalg.inv(R2),3),'\n')
        # print(np.round(np.linalg.inv(R3),3),'\n')

    def getStatus(self):
        return { 
                 'JOINTS':{
                        'J1': np.round(self.J1, 3),
                        'J2': np.round(self.J2, 3),
                        'J3': np.round(self.J3, 3),
                    },
                 'ANGLES':{
                        'A1': np.round(self.A1, 3),
                        'A2': np.round(self.A2, 3),
                        'A3': np.round(self.A3, 3),
                    }
                }
        

    
R_Angle = np.pi/2
L_Angle = -np.pi/2
pos = np.array([0,0,0])
basis = np.eye(3)

meha = Meha(pos,basis)
meha.addLeg('R1',[1, 3, 0],   R_Angle)
meha.addLeg('R2',[2, 0, 0],   R_Angle)
meha.addLeg('R3',[1, -3, 0],  R_Angle)
meha.addLeg('L1',[-1, 3, 0],  L_Angle)
meha.addLeg('L2',[-2, 0, 0],  L_Angle)
meha.addLeg('L3',[-1, -3, 0], L_Angle) 
meha.getStatus()



