from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid2 import Pid
from sys import argv
from os import listdir
from os.path import isfile, join
import csv
import time

pixy_init()
board = MultiWii('/dev/ttyUSB0')

path = '/home/pi/anti-drone-system/data'
filenames = [x for x in listdir(path) if isfile(join(path, x))]
if len(filenames) != 0:
        datanum = max([int(x[4:-4]) for x in filenames]) + 1
else:
        datanum = 0
filename = 'data' + str(datanum) + '.csv'
datafile = open(join(path, filename), 'wb')
csvwriter = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
csvwriter.writerow(['time', 'x', 'y', 'roll', 'pitch', 'thrust', 'yaw'])

blocks = BlockArray(1)

landing_thrust = 1250

roll_offset = 1500              # center roll control value
pitch_offset = 1494             # center pitch control value
thrust_offset = 1480            # center thrust control value (~hover)
yaw_offset = 1500		        # center yaw control value
pixel_x_offset = 160            # center of screen on x-axis
pixel_y_offset = 100            # center of screen on y-axis
dt = 0.02                       # 50 Hz refresh rate

R_KP = 1.0
R_KI = 0.0
R_KD = 0.0
roll_pid = Pid(R_KP, R_KI, R_KD)
roll_pid.set_limit(20)
roll_pid.set_reference(pixel_x_offset)

T_KP = .50
T_KI = 0.0
T_KD = 0.0
thrust_pid = Pid(T_KP, T_KI, T_KD)
thrust_pid.set_limit(15)
thrust_pid.set_reference(pixel_y_offset)

Y_KP = 1.0
Y_KI = 0.0
Y_KD = 0.0
yaw_pid = Pid(Y_KP, Y_KI, Y_KD)
yaw_pid.set_limit(20)
yaw_pid.set_reference(pixel_x_offset)

if len(argv) > 1 and argv[1] == 'ARM':
    board.arm()
    print 'Flight controller is ARMED.'
else:
    print 'Running script in SAFE MODE.'

program_start = time.time()

while True:
    try:
        loop_start = time.time()
        count = pixy_get_blocks(1, blocks)
        if count > 0:
            x = blocks[0].x
            y = blocks[0].y
            roll = -roll_pid.get_output(x) + roll_offset
            pitch = pitch_offset
            thrust = thrust_pid.get_output(y) + thrust_offset
            yaw = -yaw_pid.get_output(x) + yaw_offset
        else:
            x = None
            y = None
            roll = -roll_pid.get_output(pixel_x_offset) + roll_offset
            pitch = pitch_offset
            thrust = thrust_pid.get_output(pixel_y_offset) + thrust_offset
            yaw = -yaw_pid.get_output(pixel_x_offset) + yaw_offset

        data = [time.time() - program_start, x, y, roll, pitch, thrust, yaw]
        print 'time=%.2f x=%3d y=%3d roll=%4d pitch=%4d thrust=%4d yaw=%4d' % tuple([0.0 if x is None else x for x in data])
        csvwriter.writerow(data)
        command = [roll, pitch, thrust, yaw, 1500, 1500, 1500, 1500]
        board.sendCMD(16, MultiWii.SET_RAW_RC, command)
        time.sleep(dt - (loop_start - time.time()))

    except KeyboardInterrupt:
        print 'Landing mode. Press CTRL+C to stop.'
        while True:
            loop_start = time.time()
            try:
                board.sendCMD(16, MultiWii.SET_RAW_RC, [1500, 1500, landing_thrust, 1500, 1500, 1500, 1500, 1500])
                time.sleep(dt - (loop_start - time.time()))
            except KeyboardInterrupt:
                datafile.close()
                board.disarm()
                pixy_close()
                break
        break
