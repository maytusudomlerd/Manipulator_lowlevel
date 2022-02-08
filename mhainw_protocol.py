import serial

ser = serial.Serial('/dev/tty.usbmodem1414203', 256000, timeout=1)

def checksum(data):
    return ~(sum(data)) & 0xff
    
def Rx():
    Rx_buf = []
    if ser.is_open:
        Rx_buf = ser.read(3)
    if Rx_buf[0] == 0xFF:
        instuction = ARx_buf[-1]
        if instuction == 0xA1:
            return 1
        if instuction == 0xA2:
            # CRC Calculation error
            return 2
        if instuction == 0xA3:
            # Acknowledge
            return 3
        if instuction == 0xA4:
            # Done
            return 4

def tx_sethome():
    tx_buff = [0xFF,0x02,0x10,0x00]
    tx_buff[-1] = checksum(tx_buff[1:2])
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
            move_axis = 0b11111000
            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step

        elif(axis == 'y'):
            move_axis = 0b11110100
            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step

        elif(axis == 'z'):
            move_axis = 0b11110010

            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step

        elif(axis == 'rz'):
            move_axis = 0b11110001
            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step

        else:
            print('invalid axis')

        tx_buff = [0xFF,0x04,0x20,move_axis,move_step]

    elif(type == 'j'): # joint jog
        
        if(axis == 'j1'):
            move_axis = 0b11111000
            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step
            
        elif(axis == 'j2'):
            move_axis = 0b11110100
            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step
        
        elif(axis == 'j3'):
            move_axis = 0b11110010
            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step

        elif(axis == 'j4'):
            move_axis = 0b11110001
            if(step < 0):
                move_step = 0b00000000 | (-1 * step)
            else:
                move_step = 0b00010000 | step
        
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
def tx_move(position=[0,0,0,0], ref='home',type='c'):
    if(ref == 'home'):
        tx_buff = [0xff,0x00,0x30]
    elif(ref == 'current'):
        tx_buff = [0xff,0x00,0x31]
    
    if(type == 'c'):
        style = 0b11000000
    elif(type == 'j'):
        style = 0b11100000
    
    for i in range(len(position)):
        if(position[i] < 0):
            style = style | (1 << len(position) - i) #**
    tx_buff.append(style)
    for p in position:
        if(p < 0):
            p = -1*p
        tx_buff.append(round(p / 255))
        tx_buff.append(round(p % 255))
    
    tx_buff[1] = len(tx_buff) - 1
    
    tx_buff.append(checksum(tx_buff[1:]))

    print(tx_buff)
    
    if ser.is_open:
        ser.write(tx_buff)


if __name__ == "__main__":
    tx_sethome()
    # tx_jog(axis='j1', step=10 , type='j')
    # tx_move(ref='home',type='c',position=[100,-100,100,-100])
    # Rx()




