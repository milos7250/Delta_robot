import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='COM8',
    baudrate=2400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

while 1:
    # get keyboard input
    instr = input(">> ")
    if instr == 'exit':
        ser.close()
        exit()
    else:
        # send the character to the device
        alpha, beta, gamma, catch = instr.split()
        ser.write(('[' + alpha + ';' + beta + ';' + gamma + ';' + catch + ']').encode('UTF-8'))
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1).decode('UTF-8')
        if out != '':
            print(">>" + out)
