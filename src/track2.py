from pixy import *
from ctypes import *
from MultiWii import MultiWii
from Pid2 import Pid
from sys import argv
from os import listdir
from os.path import isfile, join
import csv
import time
import collections
import numpy as np
import sys

# Constants for calculating distance all sizes in mm
FOCAL_LEN = 2.8
F_STOP = 2.0
SENS_DIAG = 6.35
IMG_HEIGHT = 200.0                  # frame width pixels
IMG_WIDTH = 320.0                   # frame width pixels
SENS_RATIO = IMG_WIDTH / IMG_HEIGHT
TARG_HEIGHT = 355.6
SENS_HEIGHT = pow(pow(SENS_DIAG, 2)  / (pow(SENS_RATIO, 2) + 1), .5)
Y_FOV = 47                          # Vertical field of view %
DEG_PPX_Y = IMG_HEIGHT / Y_FOV      # % per pixel 

def parseBlock(block):
    # RETURNS :
    # [x, y, inv_size]
    #   - x, y : x & y coordnates for center of detected object
    #   - dist : the inverted size of the detected region. NOTE: This
    #       value is only assigned under the condition that the object is 
    #       entirely within the frame or that it is detected as being too 
    #       close to the pixy
    min_dist = 1300
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

    dist = (F_STOP * TARG_HEIGHT * IMG_HEIGHT) / (block.height * SENS_HEIGHT) 

    not_too_close = dist > min_dist
    out_of_x = (l_x <= xmin) or (r_x >= xmax)
    out_of_y = (t_y <= ymin) or (b_y >= ymax)
    
    if not_too_close and (out_of_x or out_of_y): 
        print 'PARTIALLY OUT OF FRAME'
        dist = None
    return [block.x, block.y, dist]

pixy_init()                             # connect camera
blocks = BlockArray(1)                  # array for camera output
board = MultiWii('/dev/ttyUSB0')        # connect arduino

# set up data logging
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
print 'filename=' + filename

# Constant control values
roll_offset = 1500              # center roll control value
pitch_offset = 1507             # center pitch value
thrust_offset = 1300            # center thrust control value (~hover)
yaw_offset = 1500		        # center yaw control value
pixel_x_offset = 160            # center of screen on x-axis
pixel_y_offset = 100            # center of screen on y-axis
dist_offset = 2100              # The output distance comes in discrete steps
landing_thrust = thrust_offset - 24

# Thrust controller values
T_KP = 1.2
T_KI = 0.04
T_KD = 1.4
thrust_pid = Pid(T_KP, T_KI, T_KD)
thrust_pid.set_limit(50)
thrust_pid.set_reference(pixel_y_offset)

# Yaw controller values
Y_KI = 0.0
Y_KD = 0.0
Y_KP = 1.0
yaw_pid = Pid(Y_KP, Y_KI, Y_KD)
yaw_pid.set_limit(20)
yaw_pid.set_reference(pixel_x_offset)

# Roll controller values
R_KP = 2.0
R_KI = 1.0
R_KD = 0.0
roll_pid = Pid(R_KP, R_KI, R_KD)
roll_pid.set_limit(20)
roll_pid.set_reference(pixel_x_offset)

# Pitch controller values
P_BUFF_LEN = 10      # Size of circular buffer for size_inv values
P_KP = 0
P_KI = 0
P_KD = 0

# Dist controller values
P_KP = 0.0045
P_KI = 0    #.0005
P_KD = 0.04

# Create buffer to act as filter for pitch values
while pixy_get_blocks(1, blocks) == 0:
    print 'Attempting to lock distance'
[x, y, dist] = parseBlock(blocks[0])
dist_offset = dist
pitch_pid = Pid(P_KP, P_KI, P_KD)
pitch_pid.set_limit(40)
pitch_pid.set_reference(dist_offset)
pitch_buff = np.full(P_BUFF_LEN, dist)
cum_pitch = sum(pitch_buff)

# Print preliminary flight information
if len(argv) > 1 and argv[1] == 'ARM':
    board.arm()
    print 'Flight controller is ARMED.'
else:
    print 'Running script in SAFE MODE.'

print [[R_KP, R_KI, R_KD], 
       [P_KP, P_KI, P_KD], 
       [T_KP, T_KI, T_KD], 
       [Y_KP, Y_KI, Y_KD]]

program_start = time.time()
dt = 0.02                       # 50 Hz refresh rate

while True:
    try:
        loop_start = time.time()
        count = pixy_get_blocks(1, blocks)
        # calculate the y offset given current pitch
        board.getData(MultiWii.ATTITUDE)
        y_off = board.attitude['angy'] / DEG_PPX_Y
        if count > 0:
            # Detection successful. Calculate axis vlues to be sent to FC
            [x, y, dist] = parseBlock(blocks[0])
            y = y - y_off
            roll = -roll_pid.get_output(x) + roll_offset
            thrust = thrust_pid.get_output(y) + thrust_offset
            yaw = -yaw_pid.get_output(x) + yaw_offset
            # Due to pixy noise, best reading of size will be the smallest buffer val 
            if dist is not None:
                cum_pitch = cum_pitch + (dist - pitch_buff[p_i])
                pitch_buff[p_i] = dist
                dist = min(pitch_buff)
                #dist = cum_pitch / P_BUFF_LEN
                pitch = -pitch_pid.get_output(dist) + pitch_offset
                p_i = (p_i + 1) % P_BUFF_LEN
            else:
                pitch = pitch_offset - 15
        else:
            dist = None
            x = None
            y = None
            size_inv = None
            roll = -roll_pid.get_output(pixel_x_offset) + roll_offset
            pitch = -pitch_pid.get_output(dist_offset) + pitch_offset
            thrust = thrust_pid.get_output(pixel_y_offset) + thrust_offset
            yaw = -yaw_pid.get_output(pixel_x_offset) + yaw_offset
            
        data = [time.time() - program_start, x, y, dist, roll, pitch, thrust, yaw]

        try:
            print 'time=%.2f x=%3d y=%3d dist=%4d roll=%4d pitch=%4d thrust=%4d yaw=%4d' % tuple([0.0 if x is None else x for x in data])
            csvwriter.writerow(data)
        except TypeError:
            print 'Bad digit in print'
        command = [roll, pitch, thrust, yaw]
        try:
            board.sendCMD(8, MultiWii.SET_RAW_RC, command)
        except:
            print 'Command not sent'
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
