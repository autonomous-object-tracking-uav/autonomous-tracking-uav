from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid import Pid
from time import sleep
from sys import argv

pixy_init()
#board = MultiWii("/dev/ttyUSB0")

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

thrust_offset = 1420
pixel_offset = 100

time = 0
dt = 0.05
pid = Pid(5, 0, 0, dt=dt)
pid.set_limit([1200-thrust_offset, 1500-thrust_offset])
pid.set_reference(pixel_offset)

if len(argv) > 1 and argv[1] == 'ARM':
	#board.arm()
	print 'Flight controller is ARMED.'
else:
	print 'Running script in SAFE MODE.'

while True:
	try:
		count = pixy_get_blocks(100, blocks)
		if count > 0:
			frame = frame + 1
			for index in range(0, count):
				thrust = pid.get_output(blocks[index].y) + thrust_offset
				print 't=%3d, frame=%3d, y=%3d, thrust=%3d' % (time, frame, blocks[index].y, thrust)
		else:
			thrust = pid.get_output(pixel_offset) + thrust_offset
			print 't=%3d, nothing detected, thrust=%3d' % (time, thrust)	
	
		data = [1500, 1500, thrust, 1500, 1500, 1500, 1500, 1500]
		#board.sendCMD(16, MultiWii.SET_RAW_RC, data)
		sleep(dt)
		time += dt

	except KeyboardInterrupt:
		while True:
			try:	
				print 'Landing mode. Press CTRL+C to disarm.'
				#board.sendCMD(16, MultiWii.SET_RAW_RC, [1500, 1500, thrust_offset-50, 1500, 1500, 1500, 1500, 1500])
				sleep(dt)
				time += dt
			except KeyboardInterrupt:
				#board.disarm()
				break
		break
