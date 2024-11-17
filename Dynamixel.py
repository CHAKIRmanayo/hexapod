from dynamixel_sdk import *

class XM430(object):
    """Class represents a registers XM430-W210-T/R

    Args:
        ID(int): dynamixel id number
    """
        
    def __init__(self, ID):
        self.ADDR_TORQUE_ENABLE = 64  # address of the register for turning on/off the moment
        self.ADDR_LED_ENABLE = 65  # address of the register for turning on/off the LED
        self.ADDR_GOAL_POSITION = 116  # address of the register for setting the goal position
        self.ADDR_FIRMWARE_VERSION = 6 # address of the register for obtaining the firmware version
        self.ADDR_DRIVE_MODE = 6 # address of the register for drive mode
        self.DXL_ID = ID # Dynamixel ID

class Control:
    """Class represents a registers XM430-W210-T/R

    Args:
        PORT(str): connected port to dynamixel bus
        BAUDRATE(int): installed boudrate on your dynamixels
        PROTOCOL_VERSION(float): installed protocol on your dynamixels

    Examples:
                >>> PORT(str): '/dev/ttyACM0'
                >>> BAUDRATE(int): 1000000
                >>> BAUDRATE(int): 2.0
                
    """

    def __init__(self, PORT='/dev/ttyACM0', BAUDRATE=1000000, PROTOCOL_VERSION=2.0):
        self.port = PORT
        self.baudrate = BAUDRATE
        self.protocolVersion = PROTOCOL_VERSION

    def connect(self):
        self.portHandler = PortHandler(self.port)

        if self.portHandler.openPort():
            print("Port is open")
        else:
            print("Port couldn't be opened")
            quit()

        if self.portHandler.setBaudRate(self.baudrate):
            print("Baudrate is set")
        else:
            print("Baudrate couldn't be set")
            quit()

        if (self.portHandler.openPort() and self.portHandler.setBaudRate(self.baudrate)):
            self.packetHandler = PacketHandler(self.protocolVersion)
            return True
        else:
            return False

    def scan(self):
        self.packetHandler = PacketHandler(self.protocolVersion)

        found_motors = []
        for motor_id in range(1, 20):
            model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, motor_id)
            
            if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                print("Found motor with ID:", motor_id)
                found_motors.append(motor_id)
                print("Model Number:", model_number)
                print("Firmware Version:", self.packetHandler.read1ByteTxRx(self.portHandler, motor_id, 6))

        print("Found motors ID:", found_motors)

    def closePort(self):
        self.portHandler.closePort()
        print("Port was closed")

    def setGoalPosition(self, dxl=list, value=list):
        for i in range(len(dxl)):
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl[i].DXL_ID, dxl[i].ADDR_GOAL_POSITION, value[i])

    def setTorqueEnable(self, dxl=list, value=list):
        for i in range(len(dxl)):
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl[i].DXL_ID, dxl[i].ADDR_TORQUE_ENABLE, value[i]) 

    def setLED(self, dxl=list, value=list):
        for i in range(len(dxl)):
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl[i].DXL_ID, dxl[i].ADDR_LED_ENABLE, value[i])

    def getDriveMode(self, dxl=list):
        for i in range(len(dxl)):
            print(self.packetHandler.read1ByteTxRx(self.portHandler, dxl[i].DXL_ID, dxl[i].ADDR_DRIVE_MODE))
        

def main():
    dxlList = []
    dxlList.append(XM430(1))
    dxlControl = Control()
    dxlControl.connect()
    dxlControl.scan() # scan dynamixel bus
    dxlControl.setLED(dxlList, [0]) # LED on
    dxlControl.setTorqueEnable(dxlList, [1]) # set torque 1
    dxlControl.setGoalPosition(dxlList, [2048]) # move to center 
    dxlControl.closePort()
        
if __name__ == "__main__":
    main()