#!/usr/bin/env python

"""test-thrust.py: Test script to send RC commands to a MultiWii Board."""

from MultiWii import MultiWii
import time, sys


if __name__ == "__main__":
	thrust = int(sys.argv[1])
	T = float(sys.argv[2])
	
	board = MultiWii("/dev/ttyUSB0")

	print(thrust,T)

	board.arm()	
	start = time.time()
	timeElapsed = 0
	sleepTime = .25
	
	data = [1500, 1500, thrust, 1500, 1500, 1500, 1500, 1500]
	while timeElapsed < T:
		if thrust > 1850 and (T - timeElapsed < .65):
			thrust = 1800
			data[2] = thrust
		board.sendCMD(16, MultiWii.SET_RAW_RC, data)
		print(timeElapsed,thrust)
		timeElapsed = time.time() - start
		time.sleep(sleepTime)		

	board.disarm()
