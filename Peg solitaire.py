from time import sleep
import serial
import numpy as n
from itertools import product

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
offsets = n.loadtxt('offsets.txt')
offsets = offsets.reshape((7, 7, 3))


coordinates_original = n.zeros((7, 7, 3))
for i, j in product(range(7), range(7)):
    coordinates_original[i][j] = vector(-7.5 + i * 2.5, -7.5 + j * 2.5, -15.5)
# offsets = n.zeros_like(coordinates_original)
global_offset = n.zeros_like(coordinates_original)
coordinates = coordinates_original + global_offset + offsets

TLH = -12  # Transition layer height
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
    # print(a, '          ', b)
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


def go_to(input_effector, catch=0):  # Moves the robot arm to the given coordinates with constant z-coordinate (TLH)
    global curr_pos
    if input_effector.size == 4:
        catch = input_effector[3]
        input_effector = input_effector[:3]
    
    repeat = int(length(vector(0, 0, TLH - curr_pos[2])) * 2.5) + 1
    movement = vector(0, 0, (TLH - curr_pos[2]) / repeat)
    for i in range(repeat):
        move_by_vector(movement, catch)
        if i < 2:
            sleep(0.5 / (i + 1))

    repeat = int(length(vector(input_effector[0], input_effector[1], TLH) - curr_pos)) + 1
    movement = (vector(input_effector[0], input_effector[1], TLH) - curr_pos) / repeat
    for i in range(repeat):
        move_by_vector(movement, catch)

    repeat = int(length(input_effector - curr_pos)) * 3 + 1
    movement = (input_effector - curr_pos) / repeat
    for i in range(repeat):
        move_by_vector(movement, catch)
        sleep(0.03)
    curr_pos.tofile('curr_pos.txt', sep=' ', format='%s')
    print(curr_pos.round(3))
    return 0


def get_coordinates(input_coordinates):  # Returns the cartesian coordinates defined by chessboard-like coordinates
    first_index = input_coordinates[0]
    second_index = int(input_coordinates[1:]) - 1
    first_index = ord(first_index) - 64 - 1
    return n.array(coordinates[first_index][second_index])


def get_indices(input_coordinates):  # Returns the indices of the input coordinates
    first_index = input_coordinates[0]
    second_index = int(input_coordinates[1:]) - 1
    first_index = ord(first_index) - 64 - 1
    return n.array([first_index, second_index])


def drop():  # Drops the ball from the current position into the closest hole
    global curr_pos
    holes = n.array([get_coordinates('B2'), get_coordinates('B6'), get_coordinates('F2'), get_coordinates('F6')])
    lenghts = n.array([length(holes[0] - curr_pos), length(holes[1] - curr_pos), length(holes[2] - curr_pos),
                       length(holes[3] - curr_pos)])
    return go_to(holes[n.where(lenghts == n.amin(lenghts))[0][0]], 1)


def do_move(input_move, double_jump=0):  # Moves the ball from position 0 to position 1, given in the input move
    if double_jump == 0:
        go_to(get_coordinates(input_move[0]), 0)
        sleep(0.5)
    go_to(get_coordinates(input_move[1]), 1)
    sleep(0.1)


def drop_from_between(input_move):  # Drops the ball from between the coordinates given in the input move
    go_to(get_coordinates('{0:c}{1:d}'.format((ord(input_move[0][0]) + ord(input_move[1][0])) // 2,
                                              (int(input_move[0][1]) + int(input_move[1][1])) // 2)), 0)
    sleep(0.5)
    drop()


def game(file):  # Plays the game from the given text file
    f = open(file + '.txt', 'r')
    moves = n.array(f.read().split())
    moves = moves.reshape((len(moves) // 2, 2))
    f.close()
    i = 0
    while i < moves.size // 2 - 1:
        to_drop = n.empty((0, 2))
        while i < moves.size // 2 - 1:
            do_move(moves[i], to_drop.size)
            to_drop = n.append(to_drop, [moves[i]], axis=0)
            i = i + 1
            if moves[i - 1][1] != moves[i][0]:
                break
        for j in to_drop:
            drop_from_between(j)
    do_move(moves[-1])
    drop_from_between(moves[-1])


def cycle(contain):  # Cycles through the positions, which include the input string
    for i in n.array('A3 A4 A5 B3 B4 B5 C1 C2 C3 C4 C5 C6 C7 D1 D2 D3 D4 D5 D6 D7 E1 E2 E3 E4 E5 E6 E7 F3 F4 F5 G3 G4'
                     ' G5'.split()):
        if len(contain) == 0:
            go_to(get_coordinates(i), 0)
            sleep(0.5)
        else:
            if contain in i:
                go_to(get_coordinates(i), 0)
                sleep(0.5)


def set_offset(input_coordinates):  # Sets the offset for given position AND writes offsets into file
    global coordinates
    while 1:
        input_vector = input('Enter the coordinates of the vector (x, y, z), by which you want to offset position {0}'
                             ' or DONE, when you are done:\r\n>> '.format(input_coordinates))
        if input_vector == 'DONE':
            index1, index2 = get_indices(input_coordinates)
            offsets[index1][index2] = offsets[index1][index2] + (curr_pos - get_coordinates(input_coordinates))
            print(offsets[index1][index2])
            f = open('offsets.txt', 'w')
            for i, j in product(range(7), range(7)):
                f.write('{0:-7.3f} {1:-7.3f} {2:-7.3f} #{3:c}{4}\n'.format(offsets[i][j][0], offsets[i][j][1],
                                                                           offsets[i][j][2], i + 64 + 1, j + 1))
                if j == 6:
                    f.write('\n')
            f.close()
            coordinates = coordinates_original + global_offset + offsets
            return 0
        go_to(curr_pos + n.array(input_vector.split(), dtype=float), 0)


def set_global_offset(with_offsetting):  # Calibrates the coordinates of holes according to four border places (G4, D7,
                                            # A4, D1)
    global offsets, coordinates, coordinates_original, global_offset
    coordinates = coordinates - offsets
    if with_offsetting == 1:
        base = n.zeros((4, 3))
        for index, i in enumerate('G4 D7 A4 D1'.split()):
            go_to(get_coordinates(i), 0)
            while 1:
                input_vector = input(
                    'Enter the coordinates of the vector (x, y, z), by which you want to offset position {0}'
                    ' or DONE, when you are done:\r\n>> '.format(i))
                if input_vector == 'DONE':
                    base[index] = curr_pos
                    break
                go_to(curr_pos + n.array(input_vector.split(), dtype=float))
    else:
        base = n.loadtxt('base.txt')
        print(base)
        base = base.reshape((4, 3))
    base = (base + vector(0, 0, 15.5)) / 3
    for i, j in product(range(4), range(4)):
        global_offset[i + 3][j + 3] = base[0] * i + base[1] * j
    for i, j in product(range(3), range(4)):
        global_offset[i][j + 3] = base[1] * j + base[2] * (3 - i)
    for i, j in product(range(3), range(3)):
        global_offset[i][j] = base[2] * (3 - i) + base[3] * (3 - j)
    for i, j in product(range(4), range(3)):
        global_offset[i + 3][j] = base[3] * (3 - j) + base[0] * i
    global_offset = global_offset - vector(0, 0, 15.5) - coordinates_original
    coordinates = coordinates_original + global_offset
    f = open('coordinates.txt', 'w')
    for i, j in product(range(7), range(7)):
        f.write('{0:-7.3f} {1:-7.3f} {2:-7.3f} #{3:c}{4}\n'.format(coordinates[i][j][0], coordinates[i][j][1],
                                                                   coordinates[i][j][2], i + 64 + 1, j + 1))
        if j == 6:
            f.write('\n')
    f.close()
    f = open('base.txt', 'w')
    for i in 'G4 D7 A4 D1'.split():
        f.write('{0:-7.3f} {1:-7.3f} {2:-7.3f} #{3}\n'.format(get_coordinates(i)[0], get_coordinates(i)[1],
                                                              get_coordinates(i)[2], i))
    f.close()
    coordinates = coordinates + offsets
    go_to(vector(0, 0, TLH))
    print(coordinates)


def print_game(file):  # Prints the solution of the game on screen and saves it into a vector graphics file
    import tkinter as t
    side = 25
    c = t.Canvas(height=side * 7.5 * 8, width=side * 7.5 * 8, bg='white')
    c.pack()
    pegs = n.array([[-1,-1,1,1,1,-1,-1], [-1,-1,1,1,1,-1,-1], [1,1,1,1,1,1,1], [1,1,1,-1,1,1,1], [1,1,1,1,1,1,1], [-1,-1,1,1,1,-1,-1], [-1,-1,1,1,1,-1,-1]])

    def print_pegs(posx, posy, moved=n.array(['A1', 'A1']), dropped=n.array([['A1', 'A3']])):
        posx = posx * 7.5 * side + 0.5 * side
        posy = posy * 7.5 * side + 0.5 * side
        for i in range(8):
            if i in [0,1,6,7]:
                c.create_line(posx + i * side, posy + side * 2, posx + i * side, posy + side * 5)
                c.create_line(posx + side * 2, posy + i * side, posx + side * 5, posy + i * side)
            else:
                c.create_line(posx + i * side, posy, posx + i * side, posy + side * 7)
                c.create_line(posx, posy + i * side, posx + side * 7, posy + i * side)
        for index, i in enumerate(pegs.flatten()):
            x = index // 7
            y = 6 - (index % 7)
            if i == 1:
                c.create_oval(posx + x * side + side // 4, posy + y * side + side // 4, posx + (x + 1) * side - side // 4, posy + (y + 1) * side - side // 4, fill='black')

        x = get_indices(moved[1])[0]
        y = 6 - get_indices(moved[1])[1]
        if pegs[x, 6 - y] == 1:
            c.create_oval(posx + x * side + side // 4, posy + y * side + side // 4, posx + (x + 1) * side - side // 4, posy + (y + 1) * side - side // 4,
                          fill='red', outline='red')

        x = get_indices(dropped[0][0])[0]
        y = 6 - get_indices(dropped[0][0])[1]
        if pegs[x, 6 - y] == 0:
            c.create_oval(posx + x * side + side // 4, posy + y * side + side // 4, posx + (x + 1) * side - side // 4, posy + (y + 1) * side - side // 4, outline='red')

        for i in dropped:
            moved = i
            x = (ord(moved[0][0]) + ord(moved[1][0])) // 2 - 64 - 1
            y = 6 - ((int(moved[0][1]) + int(moved[1][1])) // 2 - 1)
            if pegs[x, 6 - y] == 0:
                c.create_oval(posx + x * side + side // 4, posy + y * side + side // 4, posx + (x + 1) * side - side // 4, posy + (y + 1) * side - side // 4, outline='black')
        return 0

    def set_peg(move, value):
        pegs[get_indices(move)[0], get_indices(move)[1]] = value
        return 0

    f = open(file + '.txt', 'r')
    moves = n.array(f.read().split())
    moves = moves.reshape((len(moves) // 2, 2))
    f.close()
    i = 0
    count = 0
    print_pegs(count % 8, count // 8)
    count += 1
    while i < moves.size // 2 - 1:
        to_drop = n.empty((0, 2))
        while i < moves.size // 2 - 1:
            set_peg(moves[i][0], 0)
            set_peg(moves[i][1], 1)
            to_drop = n.append(to_drop, [moves[i]], axis=0)
            i = i + 1
            if moves[i - 1][1] != moves[i][0]:
                break
        for j in to_drop:
            set_peg('{0:c}{1:d}'.format((ord(j[0][0]) + ord(j[1][0])) // 2, (int(j[0][1]) + int(j[1][1])) // 2), 0)
        print_pegs(count % 8, count // 8, moved=moves[i - 1], dropped=to_drop)
        count += 1
    set_peg(moves[-1][0], 0)
    set_peg(moves[-1][1], 1)
    j = moves[-1]
    set_peg('{0:c}{1:d}'.format((ord(j[0][0]) + ord(j[1][0])) // 2, (int(j[0][1]) + int(j[1][1])) // 2), 0)
    print_pegs(count % 8, count // 8, moved=j, dropped=n.array([j]))
    count += 1

    from PIL import Image
    c.update()
    c.postscript(file=file + '.eps')
    # img = Image.open(file + '.eps')
    # img.save(file + '.png', 'png', quality=100, subsampling=0)
    c._root().destroy()
    return 0


set_global_offset(0)
go_to(vector(0, 0, TLH), 0)

skip_input = 0
while CONNECTED >= 0:  # Control interface
    if skip_input == 0:
        input_string = input('Enter coordinates in form \'A1 catch\' or D for drop, C for coordinates, V for vector, '
                             'A for angles, G for game, P to print solution, CY for cycle, O for offsetting, '
                             'GO for global offsetting, or EXIT:\r\n>> ')
        skip_input = 1
    if input_string == 'EXIT':
        if CONNECTED == 1:
            ser.close()
        exit()
    elif input_string == 'D':
        drop()
        skip_input = 0
    elif input_string == 'C':
        input_string2 = input('Enter coordinates in form \'x y z catch\' or DONE, when you are done:\r\n>> ')
        if input_string2 == 'DONE':
            skip_input = 0
        else:
            go_to(n.array(input_string2.split(), dtype=float))
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
    elif input_string == 'CY':
        input_string2 = input('Enter coordinates of places to cycle in form \'A\' or \'1\', leave blank to cycle all'
                              ' or DONE, when you are done:\r\n>> ')
        if input_string2 == 'DONE':
            skip_input = 0
        else:
            cycle(input_string2)
            go_to(vector(0, 0, TLH), 0)
    elif input_string == 'G':
        input_string2 = input('Enter the number of game you want to play:\r\n>> ')
        game('game' + input_string2)
        go_to(vector(0, 0, TLH), 0)
        skip_input = 0
    elif input_string == 'O':
        input_string2 = input('Enter coordinates of place you want to offset in form \'A1 \' or DONE, when you are '
                              'done:\r\n>> ')
        if input_string2 == 'DONE':
            skip_input = 0
        else:
            go_to(get_coordinates(input_string2), 0)
            set_offset(input_string2)
    elif input_string == 'GO':
        set_global_offset(1)
        skip_input = 0
    elif input_string == 'P':
        input_string2 = input('Enter the number of game you want to print:\r\n>> ')
        print_game('game' + input_string2)
        skip_input = 0
    else:
        if len(input_string) == 2:
            input_string = input_string + ' 0'
        go_to(get_coordinates(input_string.split()[0]), int(input_string.split()[1]))
        skip_input = 0