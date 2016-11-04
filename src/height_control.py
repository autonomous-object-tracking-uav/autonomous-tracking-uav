from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid import Pid
from time import sleep
from sys import argv

pixy_init()
board = MultiWii("/dev/ttyUSB0")

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

roll_offset = 1500
thrust_offset = 1420
pixel_x_offset = 160
pixel_y_offset = 100


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
			thrust = thrust_pid.get_output(blocks[0].y) + thrust_offset
			roll = -roll_pid.get_output(blocks[0].x) + roll_offset
			print 't=%3d, frame=%3d, x=%3d, y=%3d, size=%3d, roll=%3d, thrust=%3d' % (time, frame, blocks[0].x, blocks[0].y, blocks[0].width*blocks[0].height, roll, thrust)
		else:
			thrust = thrust_pid.get_output(pixel_y_offset) + thrust_offset
			roll = roll_pid.get_output(pixel_x_offset) + roll_offset
			print 't=%3d, nothing detected, roll=%3d, thrust=%3d' % (time, roll, thrust)	
	
		data = [roll, 1500, thrust, 1500, 1500, 1500, 1500, 1500]
		board.sendCMD(16, MultiWii.SET_RAW_RC, data)
		sleep(dt)
		time += dt

	except KeyboardInterrupt:
		while True:
			try:	
				print 'Landing mode. Press CTRL+C to disarm.'
				board.sendCMD(16, MultiWii.SET_RAW_RC, [1500, 1500, 1400, 1500, 1500, 1500, 1500, 1500])
				sleep(dt)
				time += dt
			except KeyboardInterrupt:
				board.disarm()
				break
		break
