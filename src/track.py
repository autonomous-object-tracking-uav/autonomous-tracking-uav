from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid import Pid
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
csvwriter.writerow(['time', 'frame', 'x', 'y', 'size_inv', 'roll', 'pitch', 'thrust', 'yaw'])

blocks = BlockArray(1)
frame  = 0

roll_offset = 1500              # center roll control value
pitch_offset = 1500             # center pitch control value
thrust_offset = 1250            # center thrust control value (~hover)
pixel_x_offset = 160            # center of screen on x-axis
size_inv_offset = 0.0025        # inverse of size at three paces distance
pixel_y_offset = 100            # center of screen on y-axis

dt = 0.02                       # 50 Hz refresh rate

R_KP = .1
R_KI = 0
R_KD = 0
roll_pid = Pid(R_KP, R_KI, R_KD, dt=dt)
roll_pid.set_limit([-20, 20])
roll_pid.set_reference(pixel_x_offset)

P_KP = 10000
P_KI = 0
P_KD = 0
pitch_pid = Pid(P_KP, P_KI, P_KD, dt=dt)
pitch_pid.set_limit([-20, 20])
pitch_pid.set_reference(size_inv_offset)

T_KP = 2
T_KI = 0
T_KD = 0
thrust_pid = Pid(T_KP, T_KI, T_KD, dt=dt)
thrust_pid.set_limit([-50, 50])
thrust_pid.set_reference(pixel_y_offset)

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
			frame = frame + 1
			x = blocks[0].x
			y = blocks[0].y
			size_inv = 1.0/((blocks[0].width + 1) * (blocks[0].height + 1))
			roll = -roll_pid.get_output(x) + roll_offset
			pitch = -pitch_pid.get_output(size_inv) + pitch_offset
			thrust = thrust_pid.get_output(y) + thrust_offset
			yaw = 1500
		else:
			x = None
			y = None
			size_inv = None
			roll = -roll_pid.get_output(pixel_x_offset) + roll_offset
			pitch = -pitch_pid.get_output(size_inv_offset) + pitch_offset
			thrust = thrust_pid.get_output(pixel_y_offset) + thrust_offset
			yaw = 1500

		data = [time.time() - program_start, frame, x, y, size_inv, roll, pitch, thrust, yaw]
		print data
		csvwriter.writerow(data)
		command = [roll, pitch, thrust, yaw, 1500, 1500, 1500, 1500]
		board.sendCMD(16, MultiWii.SET_RAW_RC, command)
		time.sleep(dt - (loop_start - time.time()))

	except KeyboardInterrupt:
		datafile.close()
		while True:
            loop_start = time.time()
			try:
				print 'Landing mode. Press CTRL+C to stop.'
				board.sendCMD(16, MultiWii.SET_RAW_RC, [1500, 1500, 1400, 1500, 1500, 1500, 1500, 1500])
				time.sleep(dt - (loop_start - time.time()))
			except KeyboardInterrupt:
				board.disarm()
				pixy_close()
				break
		break
