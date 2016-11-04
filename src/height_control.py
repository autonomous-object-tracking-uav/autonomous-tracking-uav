from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid import Pid
from time import sleep

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
pid = Pid(10, 1, 2, dt=dt)
pid.set_limit([1200-thrust_offset, 1500-thrust_offset])
pid.set_reference(pixel_offset)

while True:
	count = pixy_get_blocks(100, blocks)
	if count > 0:
		frame = frame + 1
		for index in range(0, count):
			thrust = pid.get_output(blocks[index].y) + thrust_offset
			print 't=%3d, frame=%3d, y=%3d, thrust=%3d' % (time, frame, blocks[index].y, thrust)
	else:
		thrust = pid.get_output(pixel_offset) + thrust_offset
		print 't=%3d, nothing detected, thrust=%3d' % (time, thrust)

	sleep(dt)
	time += dt
