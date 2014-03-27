OpenROV IMU tester by Matthew Valancy! 

Basic bare IMU board test

1) Plug in the tester to USB power.
2) Connect IMU by tilting board on the header
3) Press soft reset button (the black one) 
4) Look at lights to see if IMU is OK
5) If IMU light is red then switch the address switch on the bottom of the tester and it will retest


Testing on ROV
Same as above but use Brians cable (DB25 to 4 pin IMU header)


Want to build another one because the original got destroyed or lost?

You need 4 LEDS, 1 toggle switch, 1 push button switch, 1 arduino. 
Wire with LEDS connected to ground through 1k resistor with positive side on arduino.
Toggle switch and push button are pulled high by arduino and set low when active.
Pins are shown in the arduino code.


Good luck.

Need help? scubasonar@gmail.com


