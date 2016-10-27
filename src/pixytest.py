from pixy import *
from ctypes import *
from MultiWii import MultiWii


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
XCNTR = float(319.0/2.0)
YCNTR = float(199.0/2.0)

MAX_THRUST = float(2000)
MIN_THRUST = float(1000)
MED_THRUST = float((MAX_THRUST + MIN_THRUST)/2.0)
THRUST_SCALE = float((MAX_THRUST-MED_THRUST)/(YCNTR))

MAX_YAW = float(2000)
MIN_YAW = float(1000)
MED_YAW = float((MAX_YAW + MIN_YAW)/2.0)
YAW_SCALE = float((MAX_YAW-MED_YAW)/(XCNTR))

while True:

	# FC CODE
	#board.getData(MultiWii.ATTITUDE)
	#print board.attitude		

	# PIXY CODE
	count = pixy_get_blocks(100, blocks)
	if count > 0:
		# Blocks found #
		#print 'frame %3d:' % (frame)
		frame = frame + 1
		for index in range (0, count):
			#print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
			xDim = blocks[index].x
			yDim = blocks[index].y

			# Following block is for testing purposes			
			if xDim < XCNTR: xTest = 1
			elif xDim > XCNTR: xTest = -1
			else: xTest = 0
			if yDim < YCNTR: yTest = 1
			elif yDim > YCNTR: yTest = -1
			else: yTest = 0

			thrust = MED_THRUST + THRUST_SCALE*(-yDim + YCNTR)
			yaw = MED_YAW + YAW_SCALE*(-xDim + XCNTR)

			#print "\rX location: ", str(xDim) + " - Y location: ", yDim,	# This line is for testing above block 
			print "Yaw value: " , str(yaw) , " - Thrust value: " , str(thrust)
