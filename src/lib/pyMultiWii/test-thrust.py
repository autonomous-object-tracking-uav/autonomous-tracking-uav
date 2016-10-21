#!/usr/bin/env python

"""test-thrust.py: Test script to send RC commands to a MultiWii Board."""

from pyMultiwii import MultiWii
import time, sys


if __name__ == "__main__":
	thrust = int(sys.argv[1])
	T = float(sys.argv[2])
	
	board = MultiWii("/dev/ttyUSB0")

	board.arm()
	
	start = time.time()
	timeElapsed = 0
	sleepTime = .9

	while timeElapsed < T:
		print(timeElapsed)
		timeElapsed = time.time() - start
		time.sleep(sleepTime)	

	board.disarm()


