This project has been developed using Automake and Makefile setup from: https://github.com/sudar/Arduino-Makefile

Sleepy Pi 2 needs to have software power jumper enabled if programming from the Pi.
	i2cset -y 1 0x24 0xFD
Turn it off with:
	i2cset -y 1 0x24 0xFF

It's not strictly necessary, but Arduino-Makefile includes a script to reset the Arduino. It would need to be setup to set pin 22 of the RPi's GPIO high for about 1/3 of a second. Alternatively, https://github.com/SpellFoundry/avrdude-rpi provides a patched avrdude that handles this for you. This is the approach I have setup here. The Makefile points to a special avr_tools directory that contains this patched version. None of this is necessary if you are willing to manually hit the reset button whenever you try to upload code.
