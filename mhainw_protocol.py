import serial

ser = serial.Serial('/dev/tty.usbmodem1424203', 256000, timeout=1)

def checksum(data):
    return ~(sum(data)) & 0xff
    
def Rx():
    Rx_buf = []
    if ser.is_open:
        Rx_buf = ser.read(3)
    if Rx_buf[0] == 0xFF:
        instuction = Rx_buf[-1]
        if instuction == 0xA1:
            return 1
        if instuction == 0xA2:
            # CRC Calculation error
            return Rx_buf
        if instuction == 0xA3:
            # Acknowledge
            return 3
        if instuction == 0xA4:
            # Done
            return 4

def tx_sethome():
    tx_buff = [0xFF,0x02,0x10]
    tx_buff.append(checksum(tx_buff[1:]))
    print(tx_buff)
    if ser.is_open:
        ser.write(tx_buff)

"""

robot jog
    type = parameter which detemine type of joging 'c' catesian jog 'j' joint jog
    catesian jog 
    axis = parameter which detemine robot moving direction in catesian axis {input x y z rz}
    step = parameter which detemine robor movig step {input 1 5 10 mm }

    joint jog
    axis = parameter which detemine robot moving direction in robot joint {input j1(deg) j2(deg) j3(mm) j4(deg)}
    step = parameter which detemine robor movig step {input 1 5 10}

"""
def tx_jog(axis, step, type = 'c'):

    if(type == 'c'): # catesian jog
        if(axis == 'x'):
            move_axis = 0b00001000  #8
        elif(axis == 'y'):
            move_axis = 0b00000100  #4
        elif(axis == 'z'):
            move_axis = 0b00000010  #2
        elif(axis == 'rz'):
            move_axis = 0b00000001  #1

        if(step < 0):
            move_step = step & 0xFF
        else:
            move_step = step

        tx_buff = [0xFF,0x04,0x20,move_axis,move_step]

    elif(type == 'j'): # joint jog
        
        if(axis == 'j1'):
            move_axis = 0b00001000
        elif(axis == 'j2'):
            move_axis = 0b00000100
        elif(axis == 'j3'):
            move_axis = 0b00000010
        elif(axis == 'j4'):
            move_axis = 0b00000001

        if(step < 0):
            move_step = step & 0xFF
        else:
            move_step = step

        tx_buff = [0xFF,0x04,0x21,move_axis,move_step]

    tx_buff.append(checksum(tx_buff[1:]))
    print(tx_buff)
    if ser.is_open:
        ser.write(tx_buff)

"""
move the robot
    robot can move in 2 style 1. move to target. the target with respect to home position
                              2. move relatime to. input position will add in now position so the target position = now_position + input position
    ref = refferance target {home ,current}
    type = type of input {c(catesian) , j(joint)}
    position = list of position 1. input are catesian position {x(mm), y(mm), z(mm), rz(mm)}
                                2. input type are joint configuration {j1(deg), j2(deg), j3(mm), j4(deg)}
"""
def tx_move(position=[0,0,0,0], ref='home',type='j'):
    if(type == 'c'):
        tx_buff = [0xff,0x00,0x30]
    elif(type == 'j'):
        tx_buff = [0xff,0x00,0x31]
    
    style = 0b00000000
    
    if(ref == 'current'):
        style |= 1

    tx_buff.append(style)
    
    for i in range(len(position)):
        if(position[i] < 0):
            position[i] = position[i] & 0xFFFF
        tx_buff.append((position[i] >> 8) & 0xFF)
        tx_buff.append(position[i] & 0xFF)
         
    tx_buff[1] = len(tx_buff) - 1
    
    tx_buff.append(checksum(tx_buff[1:]))

    print(tx_buff)
    
    if ser.is_open:
        ser.write(tx_buff)

if __name__ == "__main__":
    # tx_sethome()
    # tx_jog(axis='j2', step=15 , type='j')
    # tx_jog(axis='x', step=10, type='c')
    # print(Rx())
    # tx_move(ref='current',type='j',position=[30,-60,0,0])
    # tx_move(ref='home',type='j',position=[60,-120,0,0])
    tx_move(ref='current',type='c',position=[-10,20,0,0])





