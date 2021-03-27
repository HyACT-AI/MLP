import sys
import time
import serial

import xylobot

class ComPort(xylobot.Protocol):
    
    def __init__(self):
        xylobot.Protocol.__init__(self)
        self.comport = serial.Serial()
        self.is_connected = False
    
    def open(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]

            for name in ports:
                try:
                    print('* {0} checking '.format(name), end = ' : ')
                    self.comport = serial.Serial(port=name, baudrate=115200)
                    para = self.readModeStatus()
                    if para > 0:
                        print(' Connection success!!!')
                        self.writeModeStatus(self.MODE_BASIC)
                        time.sleep(0.5)
                        self.is_connected = True
                        break
                    else:
                        self.comport.close()
                except (OSError, serial.SerialException):
                    pass

                print(' Connection fail...')
        else:
            print('This platform don\'t support.')

    def close(self):
        if self.comport.is_open == True:
            self.writeTorqueStatus(self.TORQUE_OFF, self.TORQUE_OFF, self.TORQUE_OFF)
            self.writeModeStatus(self.MODE_READY)

        self.is_connected = False
        self.comport.close()

    def readModeStatus(self):
        data = [[0], [ -1, -1, -1]]

        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_READ_MODE_STATUS)
            self.comport.write(packet)
            time.sleep(0.05)
            
            if self.comport.inWaiting() >= self.PACKET_LENGTH:
                packet = self.comport.read(self.PACKET_LENGTH)
                data = self.decodePacket(packet)
        else:
            print('Disconnect Xylobot')

        return data[1][0]

    def readPosition(self):
        data = [[0], [ -1, -1, -1]]
        
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_READ_AXES_NOW_POSITION)
            self.comport.write(packet)
            time.sleep(0.05)

            if self.comport.inWaiting() >= self.PACKET_LENGTH:
                packet = self.comport.read(self.PACKET_LENGTH)
                data = self.decodePacket(packet)
        else:
            print('Disconnect Xylobot')
            
        return data[1]
    
    def writeModeStatus(self, mode):
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_WRITE_MODE_STATUS, mode)
            self.comport.write(packet)
        else:
            print('Disconnect Xylobot')

    def writeTorqueStatus(self, torque1, torque2, torque3):
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_WRITE_AXES_TORQUE_STATUS, torque1, torque2, torque3)
            self.comport.write(packet)
        else:
            print('Disconnect Xylobot')

    def writePosition(self, axis1, axis2, axis3):
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_WRITE_AXES_GOAL_POSITION, axis1, axis2, axis3)
            self.comport.write(packet)
        else:
            print('Disconnect Xylobot')

    def writeSpeed(self, axis1, axis2, axis3):
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_WRITE_AXES_SPEED, axis1, axis2, axis3)
            self.comport.write(packet)            
        else:
            print('Disconnect Xylobot')
    
    def writeColorName(self, color):
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_WRITE_LED_COLOR_NAME, color)
            self.comport.write(packet)
        else:
            print('Disconnect Xylobot')
    
    def writeRgb(self, r, g, b):
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_WRITE_LED_COLOR_RGB, r, g, b)
            self.comport.write(packet)
        else:
            print('Disconnect Xylobot')
    
    def writePlay(self, pitch, duration):
        if self.comport.is_open == True:
            packet = self.makePacket(self.INSTRUCTION_WRITE_PLAY_NOTE, pitch)
            self.comport.write(packet)
            time.sleep(duration)
        else:
            print('Disconnect Xylobot')