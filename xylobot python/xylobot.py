import serial
import time
import math
import keyboard

WRITE_MODE_STATE = 0x41

MODE_READY = 0x01
MODE_BASIC = 0x03

WRITE_LED = 0x45

LED_OFF = 0x00
LED_RED = 0x01
LED_ORANGE = 0x02
LED_YELLOW = 0x03
LED_GREEN = 0x04
LED_BLUE = 0x05
LED_DARKBLUE = 0x06
LED_PURPLE = 0x07
LED_H_RED = 0x08
LED_WRITE = 0x09

WRITE_PLAY = 0x49

PLAY_DO = 0x01
PLAY_RE = 0x02
PLAY_MI = 0x03
PLAY_FA = 0x04
PLAY_SOL = 0x05
PLAY_RA = 0x06
PLAY_SI = 0x07
PLAY_H_DO = 0x08
PLAY_REST = 0x09

WRITE_JOINT = 0xAD

serial_constructor = serial.Serial("COM6", 115200)

def generatePacket(inst, para_1, para_2, para_3):
    buffers = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    checksum = 0

    buffers[0] = 0xFF
    buffers[1] = 0xFF
    buffers[2] = inst
    buffers[3] = (para_1 >> 8) & 0xFF
    buffers[4] = (para_1 & 0xFF)
    buffers[5] = (para_2 >> 8) & 0xFF
    buffers[6] = (para_2 & 0xFF)
    buffers[7] = (para_3 >> 8) & 0xFF
    buffers[8] = (para_3 & 0xFF)
    
    for i in range(9):
        checksum = checksum + buffers[i]
    buffers[9] = checksum & 0xFF

    return buffers

def writeMode(mode):
    packet = generatePacket(WRITE_MODE_STATE, mode, 0, 0)
    serial_constructor.write(packet)

def writeLED(color):
    packet = generatePacket(WRITE_LED, color, 0, 0)
    serial_constructor.write(packet)

def writePlay(note):
    packet = generatePacket(WRITE_PLAY, note, 0, 0)
    serial_constructor.write(packet)

def writeJoint(angle_1, angle_2, angle_3):
    A_TO_S = 0.29296875
    step1 = int((150 - angle_1) / A_TO_S)
    step2 = int((150 - angle_2) / A_TO_S)
    step3 = int((150 - angle_3) / A_TO_S)
    packet = generatePacket(WRITE_JOINT, step1, step2, step3)
    serial_constructor.write(packet)

def inversKinematics(x, y, z):
    calculation = True
    degree = [0, 0, 0]
    theta = [0, 0, 0]
    L1 = 64
    L2 = 44
    L3 = 210

    l1 = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    l2 = z - L1
    l = math.sqrt(math.pow(l1, 2) + math.pow(l2, 2))
    cos_b = (math.pow(L2, 2) + math.pow(L3, 2) - math.pow(l, 2)) / (2* L2* L3)
    cos_a = (math.pow(L2, 2) + math.pow(l, 2) - math.pow(L3, 2)) / (2 * L2 * l)
    
    if (-1 <= cos_a and cos_a <= 1) or (-1 <= cos_b and cos_b <= 1):
        b = math.atan2(math.sqrt(1 - math.pow(cos_b, 2)), cos_b)
        a = math.atan2(math.sqrt(1 - math.pow(cos_a, 2)), cos_a)
        pi = math.atan2(l2, l1)

        theta[2] = math.pi - b
        theta[1] = math.pi / 2 - pi - a
        theta[0] = math.atan2(y, x)

        for i in range(3):
            degree[i] = (theta[i] * 180) / math.pi
        degree[0] = 90 - degree[0]
        writeJoint(degree[0], degree[1], degree[2])
    else:
        calculation = False
    
    return calculation

writeMode(MODE_BASIC)
prev_coord = [0, 230, 80]
curr_coord = prev_coord
change_value = 4
inversKinematics(prev_coord[0], prev_coord[1], prev_coord[2])

while True:
    keyboard_flag = True
    if keyboard.is_pressed('q') == True:
        curr_coord[0] = prev_coord[0] + change_value
    elif keyboard.is_pressed('a') == True:
        curr_coord[0] = prev_coord[0] - change_value
    elif keyboard.is_pressed('x') == True:
        break
    else:
        keyboard_flag = False

    if keyboard_flag == True:
        flag = inversKinematics(curr_coord[0], curr_coord[1], curr_coord[2])
        if flag == True:
            prev_coord = curr_coord
            print("현재 좌표 : ", curr_coord)
    
    time.sleep(0.05)



writeMode(MODE_READY)
serial_constructor.close()