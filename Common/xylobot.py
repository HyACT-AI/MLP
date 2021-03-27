# Python 3.7.0

# 제목 : Hyact Xylobot Protocol v 1.8.2
# 날짜 : 2021.01.05
# 내용 : 일부 메소드의 자료형을 명시함.

class Protocol():    
    # Packet length
    PACKET_LENGTH = 0x0A  # ---------------------------------------------- 10
    ERROR = 0xFFFF  # ---------------------------------------------------- 65535

    # Header
    HEADER_FIRST = 0xFF  # ----------------------------------------------- 255
    HEADER_LAST = 0xFF   # ----------------------------------------------- 255

    # Instruction
    INSTRUCTION_READ_MODEL_NUMBER = 0x01  # ------------------------------ 1
    INSTRUCTION_READ_FIRMWARE_VERSION = 0x02  # -------------------------- 2
    INSTRUCTION_READ_SERIAL_PORT_BAUD_RATE = 0x03  # --------------------- 3
    INSTRUCTION_WRITE_SERIAL_PORT_BAUD_RATE = 0x04  # -------------------- 4
    INSTRUCTION_READ_BLE_PORT_BAUD_RATE = 0x05  # ------------------------ 5
    INSTRUCTION_WRITE_BLE_PORT_BAUD_RATE = 0x06  # ----------------------- 6
    INSTRUCTION_READ_AIXS_PORT_BAUD_RATE = 0x07  # ----------------------- 7
    INSTRUCTION_WRITE_AIXS_PORT_BAUD_RATE = 0x08  # ---------------------- 8
    INSTRUCTION_READ_AXES_ZERO_POSTION = 0x09  # ------------------------- 9
    INSTRUCTION_WRITE_AXES_ZERO_POSTION = 0x0A  # ------------------------ 10
    INSTRUCTION_READ_C_POSTION = 0x0B  # --------------------------------- 11
    INSTRUCTION_WRITE_C_POSTION = 0x0C  # -------------------------------- 12
    INSTRUCTION_READ_D_POSTION = 0x0D  # --------------------------------- 13
    INSTRUCTION_WRITE_D_POSTION = 0x0E  # -------------------------------- 14
    INSTRUCTION_READ_E_POSTION = 0x0F  # --------------------------------- 15
    INSTRUCTION_WRITE_E_POSTION = 0x10  # -------------------------------- 16
    INSTRUCTION_READ_F_POSTION = 0x11  # --------------------------------- 17
    INSTRUCTION_WRITE_F_POSTION = 0x12  # -------------------------------- 18
    INSTRUCTION_READ_G_POSTION = 0x13  # --------------------------------- 19
    INSTRUCTION_WRITE_G_POSTION = 0x14  # -------------------------------- 20
    INSTRUCTION_READ_A_POSTION = 0x15  # --------------------------------- 21
    INSTRUCTION_WRITE_A_POSTION = 0x16  # -------------------------------- 22
    INSTRUCTION_READ_B_POSTION = 0x17  # --------------------------------- 23
    INSTRUCTION_WRITE_B_POSTION = 0x18  # -------------------------------- 24
    INSTRUCTION_READ_HIGH_C_POSTION = 0x19  # ---------------------------- 25
    INSTRUCTION_WRITE_HIGH_C_POSTION = 0x1A  # --------------------------- 26
    INSTRUCTION_READ_P_GAIN_SETTING = 0x1B  # ---------------------------- 27
    INSTRUCTION_WRITE_P_GAIN_SETTING = 0x1C  # --------------------------- 28
    INSTRUCTION_READ_LINK_LENGTH = 0x1D  # ------------------------------- 29
    INSTRUCTION_WRITE_LINK_LENGTH = 0x1E  # ------------------------------ 30
    INSTRUCTION_READ_MAXIMUM_HORIZONTAL_MOVEMENT_TIME = 0x1F  # ---------- 31
    INSTRUCTION_WRITE_MAXIMUM_HORIZONTAL_MOVEMENT_TIME = 0x20  # --------- 32
    INSTRUCTION_READ_MAXIMUM_VERTICAL_MOVEMENT_TIME = 0x21  # ------------ 33
    INSTRUCTION_WRITE_MAXIMUM_VERTICAL_MOVEMENT_TIME = 0x22  # ----------- 34
    INSTRUCTION_READ_STANDBY_HORIZONTAL_POSTION_OFFSET = 0x23  # --------- 35
    INSTRUCTION_WRITE_STANDBY_HORIZONTAL_POSTION_OFFSET = 0x24  # -------- 36
    INSTRUCTION_READ_STANDBY_VERTICAL_POSTION_OFFSET = 0x25  # ----------- 37
    INSTRUCTION_WRITE_STANDBY_VERTICAL_POSTION_OFFSET = 0x26  # ---------- 38
    INSTRUCTION_READ_GOAL_HORIZONTAL_POSTION_OFFSET = 0x27  # ------------ 39
    INSTRUCTION_WRITE_GOAL_HORIZONTAL_POSTION_OFFSET = 0x28  # ----------- 40
    INSTRUCTION_READ_GOAL_VERTICAL_POSTION_OFFSET = 0x29  # -------------- 41
    INSTRUCTION_WRITE_GOAL_VERTICAL_POSTION_OFFSET = 0x2A  # ------------- 42
    INSTRUCTION_READ_MINIMUM_POSTION = 0x2B  # --------------------------- 43
    INSTRUCTION_WRITE_MINIMUM_POSITION = 0x2C  # ------------------------- 44
    INSTRUCTION_READ_MAXIMUM_POSITION = 0x2D  # -------------------------- 45
    INSTRUCTION_WRITE_MAXIMUM_POSITION = 0x2E  # ------------------------- 46

    INSTRUCTION_READ_MODE_STATUS = 0x40  # ------------------------------- 64
    INSTRUCTION_WRITE_MODE_STATUS = 0x41  # ------------------------------ 65
    INSTRUCTION_READ_LED_COLOR_RGB = 0x42  # ----------------------------- 66
    INSTRUCTION_WRITE_LED_COLOR_RGB = 0x43  # ---------------------------- 67
    INSTRUCTION_READ_LED_COLOR_NAME = 0x44  # ---------------------------- 68
    INSTRUCTION_WRITE_LED_COLOR_NAME = 0x45  # --------------------------- 69
    INSTRUCTION_READ_LED_STATUS = 0x46  # -------------------------------- 70
    INSTRUCTION_WRITE_LED_STATUS = 0x47  # ------------------------------- 71
    INSTRUCTION_READ_PLAY_NOTE = 0x48  # --------------------------------- 72
    INSTRUCTION_WRITE_PLAY_NOTE = 0x49  # -------------------------------- 73
    INSTRUCTION_READ_NOTE_READY_POSITION = 0x4A  # ----------------------- 74
    INSTRUCTION_WRITE_NOTE_READY_POSITION = 0x4B  # ---------------------- 75
    INSTRUCTION_READ_NOTE_GOAL_POSITION = 0x4C  # ------------------------ 76
    INSTRUCTION_WRITE_NOTE_GOAL_POSITION = 0x4D  # ----------------------- 77

    INSTRUCTION_READ_AXES_ID = 0xA0  # ----------------------------------- 160
    INSTRUCTION_WRITE_AXES_ID = 0xA1  # ---------------------------------- 161
    INSTRUCTION_READ_AXES_BAUD_RATE = 0xA2  # ---------------------------- 162
    INSTRUCTION_WRITE_AXES_BAUD_RATE = 0xA3  # --------------------------- 163
    INSTRUCTION_READ_AXES_TORQUE_STATUS = 0xA4  # ------------------------ 164
    INSTRUCTION_WRITE_AXES_TORQUE_STATUS = 0xA5  # ----------------------- 165
    INSTRUCTION_READ_AXIS_1_TORQUE_STATUS = 0xA6  # ---------------------- 166
    INSTRUCTION_WRITE_AXIS_1_TORQUE_STATUS = 0xA7  # --------------------- 167
    INSTRUCTION_READ_AXIS_2_TORQUE_STATUS = 0xA8  # ---------------------- 168
    INSTRUCTION_WRITE_AXIS_2_TORQUE_STATUS = 0xA9  # --------------------- 169
    INSTRUCTION_READ_AXIS_3_TORQUE_STATUS = 0xAA  # ---------------------- 170
    INSTRUCTION_WRITE_AXIS_3_TORQUE_STATUS = 0xAB  # --------------------- 171
    INSTRUCTION_READ_AXES_GOAL_POSITION = 0xAC  # ------------------------ 172
    INSTRUCTION_WRITE_AXES_GOAL_POSITION = 0xAD  # ----------------------- 173
    INSTRUCTION_READ_AXIS_1_GOAL_POSITION = 0xAE  # ---------------------- 174
    INSTRUCTION_WRITE_AXIS_1_GOAL_POSITION = 0xAF  # --------------------- 175
    INSTRUCTION_READ_AXIS_2_GOAL_POSITION = 0xB0  # ---------------------- 176
    INSTRUCTION_WRITE_AXIS_2_GOAL_POSITION = 0xB1  # --------------------- 177
    INSTRUCTION_READ_AXIS_3_GOAL_POSITION = 0xB2  # ---------------------- 178
    INSTRUCTION_WRITE_AXIS_3_GOAL_POSITION = 0xB3  # --------------------- 179
    INSTRUCTION_READ_AXES_SPEED = 0xB4  # -------------------------------- 180
    INSTRUCTION_WRITE_AXES_SPEED = 0xB5  # ------------------------------- 181
    INSTRUCTION_READ_AXIS_1_SPEED = 0xB6  # ------------------------------ 182
    INSTRUCTION_WRITE_AXIS_1_SPEED = 0xB7  # ----------------------------- 183
    INSTRUCTION_READ_AXIS_2_SPEED = 0xB8  # ------------------------------ 184
    INSTRUCTION_WRITE_AXIS_2_SPEED = 0xB9  # ----------------------------- 185
    INSTRUCTION_READ_AXIS_3_SPEED = 0xBA  # ------------------------------ 186
    INSTRUCTION_WRITE_AXIS_3_SPEED = 0xBB  # ----------------------------- 187
    INSTRUCTION_READ_AXES_P_GAIN = 0xBC  # ------------------------------- 188
    INSTRUCTION_WRITE_AXES_P_GAIN = 0xBD  # ------------------------------ 189
    INSTRUCTION_READ_AXIS_1_P_GAIN = 0xBE  # ----------------------------- 190
    INSTRUCTION_WRITE_AXIS_1_P_GAIN = 0xBF  # ---------------------------- 191
    INSTRUCTION_READ_AXIS_2_P_GAIN = 0xC0  # ----------------------------- 192
    INSTRUCTION_WRITE_AXIS_2_P_GAIN = 0xC1  # ---------------------------- 193
    INSTRUCTION_READ_AXIS_3_P_GAIN = 0xC2  # ----------------------------- 194
    INSTRUCTION_WRITE_AXIS_3_P_GAIN = 0xC3  # ---------------------------- 195
    INSTRUCTION_READ_AXES_TORQUE = 0xC4  # ------------------------------- 196
    INSTRUCTION_WRITE_AXES_TORQUE = 0xC5  # ------------------------------ 197
    INSTRUCTION_READ_AXIS_1_TORQUE = 0xC6  # ----------------------------- 198
    INSTRUCTION_WRITE_AXIS_1_TORQUE = 0xC7  # ---------------------------- 199
    INSTRUCTION_READ_AXIS_2_TORQUE = 0xC8  # ----------------------------- 200
    INSTRUCTION_WRITE_AXIS_2_TORQUE = 0xC9  # ---------------------------- 201
    INSTRUCTION_READ_AXIS_3_TORQUE = 0xCA  # ----------------------------- 202
    INSTRUCTION_WRITE_AXIS_3_TORQUE = 0xCB  # ---------------------------- 203
    INSTRUCTION_READ_AXES_NOW_POSITION = 0xCC  # ------------------------- 204
    INSTRUCTION_READ_AXIS_1_NOW_POSITION = 0xCD  # ----------------------- 205
    INSTRUCTION_READ_AXIS_2_NOW_POSITION = 0xCE  # ----------------------- 206
    INSTRUCTION_READ_AXIS_3_NOW_POSITION = 0xCF  # ----------------------- 207
    INSTRUCTION_READ_AXES_NOW_SPEED = 0xD0  # ---------------------------- 208
    INSTRUCTION_READ_AXES_NOW_LOAD = 0xD4  # ----------------------------- 212
    INSTRUCTION_READ_AXES_NOW_VOLTAGE = 0xD8  # -------------------------- 216

    # Parameters value
    MODE_READY = 0x01
    MODE_DEMO = 0x02
    MODE_BASIC = 0x03
    MODE_ENTRY = 0x05
    MODE_BLE = 0x07
    MODE_DEMO_READY = 0x08
    MODE_DEBUG = 0x09
    MODE_SET = 0x0A

    COLOR_OFF = 0x00
    COLOR_RED = 0x01
    COLOR_ORANGE = 0x02
    COLOR_YELLOW = 0x03
    COLOR_GREEN = 0x04
    COLOR_BLUE = 0x05
    COLOR_NAVY = 0x06
    OLOR_PURPLE = 0x07
    COLOR_HIGH_RED = 0x08
    COLOR_WHITE = 0x09

    NOTE_C = 0x01
    NOTE_D = 0x02
    NOTE_E = 0x03
    NOTE_F = 0x04
    NOTE_G = 0x05
    NOTE_A = 0x06
    NOTE_B = 0x07
    NOTE_HIGH_C = 0x08
    NOTE_REST = 0x09

    LED_ON = 0x00
    LED_READY = 0x01
    LED_TWINKLE = 0x02

    TORQUE_OFF = 0x00
    TORQUE_ON = 0x01

    def calculateCheckSum(self, buffers: list):
        i = 0
        check_sum = 0

        while i < 9:
            check_sum = check_sum + buffers[i]
            i += 1

        return check_sum & 0xFF

    def makePacket(self, instruction, parameter1=0, parameter2=0, parameter3=0):
        buffers = []
        buffers.append(Protocol.HEADER_FIRST)
        buffers.append(Protocol.HEADER_LAST)
        buffers.append(instruction)
        buffers.append((parameter1 >> 8) & 0xFF)
        buffers.append(parameter1 & 0xFF)
        buffers.append((parameter2 >> 8) & 0xFF)
        buffers.append(parameter2 & 0xFF)
        buffers.append((parameter3 >> 8) & 0xFF)
        buffers.append(parameter3 & 0xFF)
        buffers.append(self.calculateCheckSum(buffers))
        return buffers

    def decodePacket(self, packet: list):
        process = False
        instruction = Protocol.ERROR
        parameter = [ Protocol.ERROR, Protocol.ERROR, Protocol.ERROR]
        
        if packet[0] == Protocol.HEADER_FIRST and packet[1] == Protocol.HEADER_LAST:
            if packet[9] == self.calculateCheckSum(packet):
                if 0x01 <= packet[2] and packet[2] <= 0x2E:
                    process = True
                elif 0x40 <= packet[2] and packet[2] <= 0x4D:
                    process = True
                elif 0xA0 <= packet[2] and packet[2] <= 0xD0:
                    process = True
                elif packet[2] == 0xD4 or packet[2] == 0xD8:
                    process = True
        
        if process == True:
            instruction = packet[2]
            parameter[0] = (packet[3] << 8) | packet[4]
            parameter[1] = (packet[5] << 8) | packet[6]
            parameter[2] = (packet[7] << 8) | packet[8]

        return instruction, parameter

class Parameters():
    
    WEIGHT = 0.29296875
    ERROR = 65535

    AXIS_1 = 0
    AXIS_2 = 1
    AXIS_3 = 2

    def __init__(self, link1=64, link2=44, link3=210):
        self.link1 = link1
        self.link2 = link2
        self.link3 = link3        

    def positionToAngle(self, index, position):
        angle = Parameters.ERROR
        if index == Parameters.AXIS_1:
            angle = (position - 512) * Parameters.WEIGHT
        elif index == Parameters.AXIS_2:
            angle = (512 - position) * Parameters.WEIGHT
        elif index == Parameters.AXIS_3:
            angle = (512 - position) * Parameters.WEIGHT

        return angle

    def angleToPosition(self, index, angle):
        position = Parameters.ERROR
        if index == Parameters.AXIS_1:
            position = (angle / Parameters.WEIGHT) + 512
        elif index == Parameters.AXIS_2:
            position = 512 - (angle / Parameters.WEIGHT)
        elif index == Parameters.AXIS_3:
            position = 512 - (angle / Parameters.WEIGHT)

        return position