"-ARDUINO DOCUMENTATION-
By Josh Overbeek

1: SETUP
To connect the Arduino to the Pixy Camera, plug the wires in so the strip is facing away from the center of the Arduino

Our Arduino is a Mega 2560. When using the Arduino IDE, make sure you are operating on the correct type

To connect the Arduino to the RoboRio, plug the SDA (I2C) on the Rio into port 20 (SDA) on the Arduino, plug the SCL into port 21
on the Arduino, and connect the ground to any ground port on the Arduino. Don't bother plugging in the 3.3V

To provide power to the Arduino, we connect it to the Rio via the Serial Port (USB to Arduino Port cable)

To connect the Pixy Cam to the computer for PixyMon, use a Mini to USB cable. NOTE: The Pixy can only output to one place at a time.
The Pixy must be unplugged from the computer in order to send data to the Arduino

Make sure that when you remove the lens cap from the Pixy, you DO NOT TWIST THE LENS. In the eventuality that the lens is twisted,
you have to refocus the camera in PixyMon

2: PixyMon
To use PixyMon, you must have a Pixy plugged in. See setup for details
To track the Power Cube, we use signature 3
To adjust the focus, twist the lens on the Pixy until the video feed is in focus
These are the setting our Pixy currently uses
- Max Blocks 2
- Max Blocks per Signature 1
- Min block area 20 (Default)
- Min Saturation 
- Hue Spread
- Saturation Spread
- Data out port 0 (Default)
- I2C address 0x54 (Default)
- UART baudrate 19200 (Default)
- Default Program 0 (Default)
- Brightness

3: Arduino
The Arduino will run whatever program is on it as soon as it is powered on. In order to view the prints when plugged into 
the Arduino, go to "Tools", then "Serial Monitor" 
NOTE: Make sure that the Arduino is on COM3. To change this, go to "Tools", then click on the port.

4: The Prints
When connected to the Arduino, the prints will read in this order: Number of blocks, then X | Y | Area" 
