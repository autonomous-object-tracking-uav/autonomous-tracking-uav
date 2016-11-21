from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid2 import Pid
from sys import argv
from os import listdir
from os.path import isfile, join
import csv
import time

def parseBlock(block):
    # RETURNS :
    # [x, y, inv_size]
    #   - x, y : x & y coordnates for center of detected object
    #   - inv_size : the inverted size of the detected region. NOTE: This
    #   value
    #   is only assigned under the condition that the object is entirely
    #   within
    #   the frame or that it is detected as being too close to the pixy
    size_thr = 0.0002 # Distance of about 3 feet from pixy with paper
    margin = 4 # Number of pixels from edge that will signal out of frame
             
    # Data range for pixy adjusted for the margin declaring out of frame
    xmin = 1 + margin
    ymin = 1 + margin
    xmax = 319 - margin
    ymax = 198 - margin
         
    # edge boundaries of block
    l_x = block.x - (block.width / 2);    #left
    r_x = block.x + (block.width / 2);    #right
    t_y = block.y - (block.height / 2);   #top
    b_y = block.y + (block.height / 2);   #bottom

    inv_size = 1.0 / ((block.width + 1) * (block.height + 1))
    
    not_too_close = inv_size > size_thr
    out_of_x = (l_x <= xmin) or (r_x >= xmax)
    out_of_y = (t_y <= ymin) or (b_y >= ymax)
    
    if not_too_close and (out_of_x or out_of_y): 
        inv_size = None

    return [block.x, block.y, inv_size]

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
csvwriter.writerow(['time', 'x', 'y', 'size_inv', 'roll', 'pitch', 'thrust', 'yaw'])

blocks = BlockArray(1)

roll_offset = 1500              # center roll control value
pitch_offset = 1502             # center pitch control value
thrust_offset = 1300            # center thrust control value (~hover)
yaw_offset = 1500		        # center yaw control value
pixel_x_offset = 160            # center of screen on x-axis
pixel_y_offset = 100            # center of screen on y-axis
size_inv_offset = 0.003         # inverse of target size at ~2 meters distance
landing_thrust = thrust_offset - 25

dt = 0.02                       # 50 Hz refresh rate

R_KP = 0.2
R_KI = 0.015
R_KD = 2.2
roll_pid = Pid(R_KP, R_KI, R_KD)
roll_pid.set_limit(20)
roll_pid.set_reference(pixel_x_offset)

#P_KP = 1000
#P_KI = 50
#P_KD = 2200
P_KP = 0
P_KI = 0
P_KD = 0
pitch_pid = Pid(P_KP, P_KI, P_KD)
pitch_pid.set_limit(2)
pitch_pid.set_reference(size_inv_offset)

T_KP = 1.2
T_KI = 0.04
T_KD = 1.4
thrust_pid = Pid(T_KP, T_KI, T_KD)
thrust_pid.set_limit(50)
thrust_pid.set_reference(pixel_y_offset)

#Y_KP = 1.0
Y_KP = 0.0
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
pitch = pitch_offset  #TODO added for pitch hold testing
while True:
    try:
        loop_start = time.time()
        count = pixy_get_blocks(1, blocks)
        if count > 0:
            [x, y, size_inv_t] = parseBlock(blocks[0])
            size_inv = 1.0 / ((blocks[0].width + 1) * (blocks[0].height + 1))
            roll = -roll_pid.get_output(x) + roll_offset
            pitch = -pitch_pid.get_output(size_inv) + pitch_offset
            thrust = thrust_pid.get_output(y) + thrust_offset
            yaw = -yaw_pid.get_output(x) + yaw_offset
#            pitch = pitch_offset if pitch!=pitch_offset else pitch+1
#TODO SAVE FOR TESTING PITCH C            if size_inv_t is None:
#TODO                print 'OUT OF BOUNDS'
        else:
            x = None
            y = None
            size_inv = None
            roll = -roll_pid.get_output(pixel_x_offset) + roll_offset
            pitch = -pitch_pid.get_output(size_inv_offset) + pitch_offset
            if pitch != pitch_offset:
                pitch = pitch_offset
            else:
                pitch = pitch_offset+1
            thrust = thrust_pid.get_output(pixel_y_offset) + thrust_offset
            yaw = -yaw_pid.get_output(pixel_x_offset) + yaw_offset

        data = [time.time() - program_start, x, y, size_inv, roll, pitch, thrust, yaw]
        print 'time=%.2f x=%3d y=%3d size_inv=%.4f roll=%4d pitch=%4d thrust=%4d yaw=%4d' % tuple([0.0 if x is None else x for x in data])
        csvwriter.writerow(data)
        command = [roll, pitch, thrust, yaw]
        board.sendCMD(8, MultiWii.SET_RAW_RC, command)
        time.sleep(dt - (loop_start - time.time()))

    except KeyboardInterrupt:
        print 'Landing mode. Press CTRL+C to stop.'
        while True:
            loop_start = time.time()
            try:
                board.sendCMD(16, MultiWii.SET_RAW_RC, [roll_offset,
                                                        pitch_offset,
                                                        landing_thrust,
                                                        yaw_offset, 
                                                        1500, 
                                                        1500, 
                                                        1500, 
                                                        1500])
                time.sleep(dt - (loop_start - time.time()))
            except KeyboardInterrupt:
                datafile.close()
                board.disarm()
                pixy_close()
                break
        break
