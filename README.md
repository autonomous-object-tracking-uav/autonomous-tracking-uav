# anti-drone-system
The software repository for Rob Squad's autonomous tracking and capture drone system for Linux based systems.

This project has been developed in Python for the following configuration :
Raspberry Pi 3b running Raspbian Jessie
Flight Controller : Flip 32
Camera : Pixy_CMU_5 2.0.19

## Installation
1. Install the required Python libraries if you don't already have them
`$ sudo apt-get install python3 python3-scipy python3-numpy`
2. Install Pixy_CMU_5 Dependencies :
	Drivers :  http://www.cmucam.org/projects/cmucam5/wiki/Installing_PixyMon_on_Linux
	Python Library : http://www.cmucam.org/projects/cmucam5/wiki/Building_libpixyusb_as_a_Python_module_on_Linux

Additional Set Up Information That Might Be Useful
Set up RPi as a wireless access point for control with ssh :
https://frillip.com/using-your-raspberry-pi-3-as-a-wifi-access-point-with-hostapd/

## Relevant Documentation
1. [Cleanflight](https://github.com/cleanflight/cleanflight/tree/master/docs)
2. [Hackflight Parser](https://github.com/simondlevy/hackflight/tree/master/parser)

## Usage


## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History

TODO: Write history

## Credits
Oliver Scott
Samuel Tanner Lindemer
Robert LeBel

## License

TODO: Write license
