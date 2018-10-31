Arduino code for Teensy for group 18gr1032

In order to use the Teensy install:
https://www.pjrc.com/teensy/teensyduino.html

This is straight forward on Windows, but might be more problematic on other OS

Check that both power cables are plugged in and turned on.

Move the cart to the left endstop with the pendulum hanging straight down

Upload the code .ino code to the Teensy through the arduino ide.

Make sure stop switch is not pushed down.

Program has some different options, type the number in terminal where "newline" is sent along
	0 = Stop motors
	1 = Sliding mode control with constant beta
	2 = Sliding mode control with varying beta
	3 = Machine learning control which does balance for more than a couple of seconds
	r = Reset the measurements and filters, the cart should be at the left end stop with pendulum hanging straight down
	ref=X.X 	When a sliding mode controller is active, change the cart reference with for instance "ref=0.6" to change to the cart position 0.6 m
	
When starting either a controller, the cart should manually be moved to the within plus-minus 10 cm of the middle of the rail with the pendulum pointing up.

As safety the controllers will disable if the angle of the pendulum gets too large.
