from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid import Pid
from time import sleep
from sys import argv
from os import listdir
from os.path import isfile, join
import csv

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
csvwriter = csv.writer(datafile, delimiter=',')

class Blocks (Structure):
        _fields_ = [ ("type", c_uint),
                ("signature", c_uint),
                ("x", c_uint),
                ("y", c_uint),
                ("width", c_uint),
                ("height", c_uint),
                ("angle", c_uint) ]

blocks = BlockArray(100)
frame  = 0

roll_offset = 1500              # center roll value
thrust_offset = 1420            # center thrust value (~hover)
pixel_x_offset = 160            # center of screen on x-axis
pixel_y_offset = 100            # center of screen on y-axis

time = 0
dt = 0.05

roll_pid = Pid(0.1, 0, 0, dt=dt)
roll_pid.set_limit([-20, 20])
roll_pid.set_reference(pixel_x_offset)
thrust_pid = Pid(1, 0, 0, dt=dt)
thrust_pid.set_limit([-50, 50])
thrust_pid.set_reference(pixel_y_offset)

if len(argv) > 1 and argv[1] == 'ARM':
	board.arm()
	print 'Flight controller is ARMED.'
else:
	print 'Running script in SAFE MODE.'

while True:
	try:
		count = pixy_get_blocks(100, blocks)
		if count > 0:
			frame = frame + 1
			x = blocks[0].x
			y = blocks[0].y
			size = blocks[0].width * blocks[0].height
			roll = -roll_pid.get_output(x) + roll_offset
			pitch = 1500
			thrust = thrust_pid.get_output(y) + thrust_offset
			yaw = 1500
		else:
			x = None
			y = None
			size = None
			roll = roll_pid.get_output(pixel_x_offset) + roll_offset
			pitch = 1500
			thrust = thrust_pid.get_output(pixel_y_offset) + thrust_offset
			yaw = 1500

		data = [time, frame, x, y, size, roll, pitch, thrust, yaw]
		print data
		csvwriter.writerow(data)
		command = [roll, pitch, thrust, yaw, 1500, 1500, 1500, 1500]
		board.sendCMD(16, MultiWii.SET_RAW_RC, command)
		sleep(dt)
		time += dt

	except KeyboardInterrupt:
		datafile.close()
		while True:
			try:
				print 'Landing mode. Press CTRL+C to stop.'
				board.sendCMD(16, MultiWii.SET_RAW_RC, [1500, 1500, 1400, 1500, 1500, 1500, 1500, 1500])
				sleep(dt)
				time += dt
			except KeyboardInterrupt:
				board.disarm()
				break
		break
