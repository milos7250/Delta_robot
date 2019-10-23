from time import sleep
import serial
import numpy as n

ser = ''


def connect():
    global ser
    try:
        ser = serial.Serial(
            port='COM8',
            baudrate=2400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        ser.isOpen()
    except:
        print('Could not connect, check if the port number is correct.')
        print('Retrying...')
        connect()
    return 0


CONNECTED = int(input('Connect via Bluetooth? Enter 0 for NO, 1 for Yes.\r\n>>'))
# CONNECTED = 0
if CONNECTED == 1:
    connect()
    print('Connected')
else:
    print('Not Connected')


def vector(aa, bb, cc):
    return n.array([aa, bb, cc])


global_angle_offset = n.array([109, 106, 108])
motor_A = vector(-2.7, 0, 0)
r1 = 8.5
r2 = 17.1
curr_pos = n.fromfile('curr_pos.txt', sep=' ', dtype=float)
print(curr_pos)


def rotate_z(input_vector, angle):  # Returns the given vector rotated by an angle (in degrees) around the z axis
    angle = angle / 180 * n.pi
    return n.matmul(n.array([[n.cos(angle), n.sin(angle), 0], [-n.sin(angle), n.cos(angle), 0], [0, 0, 1]]),
                    input_vector)


def rotate_x(input_vector, angle):  # Returns the given vector rotated by an angle (in degrees) around the x axis
    angle = angle / 180 * n.pi
    return n.matmul(n.array([[1, 0, 0], [0, n.cos(angle), n.sin(angle)], [0, -n.sin(angle), n.cos(angle)]]),
                    input_vector)


def length(input_vector):  # Returns the length of the given vector
    return n.sqrt(n.vdot(input_vector, input_vector))


# For description of the variables in the next function, see picture
def get_angle(input_motor_effector):  # Returns the angle between the arm of the robot and the XY plane
    r2_p = n.sqrt(r2 ** 2 - input_motor_effector[1] ** 2)
    input_motor_effector[1] = 0
    d = length(input_motor_effector - motor_A)
    a = (r1 ** 2 - r2_p ** 2 + d ** 2) / (2 * d)
    h = n.sqrt(r1 ** 2 - a ** 2)
    p1 = motor_A + a / d * (input_motor_effector - motor_A)
    p2 = n.array([
        p1[0] + h / d * (input_motor_effector - motor_A)[2],
        0,
        p1[2] - h / d * (input_motor_effector - motor_A)[0]
    ])
    return n.arctan2((p2 - motor_A)[2], -(p2 - motor_A)[0]) / n.pi * 180


def get_angles(input_effector):  # Returns the angles between the arms of the robot and the XY plane
    output_angles = n.array([0., 0., 0.])
    for i in range(3):
        motor_effector = input_effector - vector(3.4, 0, 0)
        output_angles[i] = get_angle(motor_effector)
        input_effector = rotate_z(input_effector, 120)
        # print(output_angles)
    return output_angles


def send_angles(input_angles, catch=0):  # Sends a message to the robot with the angles in the necessary format
    if CONNECTED <= 0:
        return 0
    if input_angles.size == 4:
        catch = input_angles[3]
        input_angles = input_angles[:3]
    input_angles = (input_angles + global_angle_offset).round(0).astype(int).astype(str)
    ser.write(('[' + input_angles[0] + ';' + input_angles[1] + ';' + input_angles[2] + ';' + str(catch) + ']'
               ).encode('UTF-8'))
    # print('[' + input_angles[0] + ';' + input_angles[1] + ';' + input_angles[2] + ';' + str(int(catch)) + ']')
    sleep(0.05)
    a = b = ''
    while ser.inWaiting() > 0:
        message = ser.read(1)
        a = a + ' ' + str(message)
        b = b + ' ' + '{0:c}'.format(ord(message))
    with open('log.txt', 'a') as log:
        print(a, file=log)
    return 0


def move_to(input_effector, catch=0):  # Moves the robot arm to input coordinates
    global curr_pos
    if input_effector.size == 4:
        catch = input_effector[3]
        input_effector = input_effector[:3]
    
    if CONNECTED <= 0:
        curr_pos = input_effector
        get_angles(input_effector)
        return 0
    send_angles(get_angles(input_effector), catch)
    curr_pos = input_effector
    return 0


def move_by_vector(input_vector, catch=0):  # Moves the robot arm by the input vector
    global curr_pos
    if input_vector.size == 4:
        catch = input_vector[3]
        input_vector = input_vector[:3]
    move_to(curr_pos + input_vector, catch)
    return 0


skip_input = 0
while CONNECTED >= 0:  # Control interface
    if skip_input == 0:
        input_string = input('Enter C for coordinates, V for vector, A for angles or EXIT:\r\n>> ')
        skip_input = 1
    if input_string == 'EXIT':
        if CONNECTED == 1:
            ser.close()
        exit()
    elif input_string == 'C':
        input_string2 = input('Enter coordinates in form \'x y z catch\' or DONE, when you are done:\r\n>> ')
        if input_string2 == 'DONE':
            skip_input = 0
        else:
            move_to(n.array(input_string2.split(), dtype=float))
    elif input_string == 'V':
        input_string2 = input('Enter vector coordinates in form \'x y z catch\' or DONE, when you are done:\r\n>> ')
        if input_string2 == 'DONE':
            skip_input = 0
        else:
            move_by_vector(n.array(input_string2.split(), dtype=float))
    elif input_string == 'A':
        input_string2 = input('Enter angles in form \'a1 a2 a3 catch\' or DONE, when you are done:\r\n>> ')
        if input_string2 == 'DONE':
            skip_input = 0
        else:
            send_angles(n.array(input_string2.split(), dtype=float))
    else:
        print('Wrong input format')
        skip_input = 0
