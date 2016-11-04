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

		timeRemaining = T - timeElapsed
		if thrust >= 1400 and timeRemaining < 1.0:
			thrust = 1420
			data[2] = thrust
		elif thrust >= 1400 and T >= 3.0 and timeElapsed > 1.0:
			thrust = 1440
			data[2] = thrust


		board.sendCMD(16, MultiWii.SET_RAW_RC, data)
		print(timeElapsed,thrust)
		timeElapsed = time.time() - start
		time.sleep(sleepTime)		

	board.disarm()
