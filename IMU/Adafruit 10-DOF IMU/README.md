## Background

A little series on integrating an Inertial Measurement Unit (IMU) onto the I2C GPIO pins of the NVIDIA Jetson TK1. First, we’ll build an interface library, RTIMULib, to enable the IMU to talk to the Jetson. Look here: https://www.youtube.com/watch?v=gDewXVN5a-o. </br>
An inertial measurement unit (IMU) is an electronic device that measures and reports a craft’s velocity, orientation, and gravitational forces, using a combination of accelerometers and gyroscopes, sometimes also magnetometers.
The Inertial measurement unit and the optical system (scanning telescope and sextant) mounted on the precision navigation base which maintains accurate angular orientation between the two subsystems. The optical system is used to align the inertial system and for navigation in earth oribt, lunar orbit, and in cislunar space. The inertial measurement unit is used as a primary attitude reference and is used for guidance purposes during all maneuvers and during reentry.

## IMU with Jetson TK1
Unfortunately the Jetson does not have a built in IMU, but that’s easy to rectify. First we need an appropriate IMU. There are all sorts, but ideally it would have 10 Degrees Of Freedom (DOF), that is, accelerometer, magnetometer, and gyroscope, along with a barometer pressure sensor. 10 DOF is a bit of a misnomer, but a cool name nonetheless. A lot of these devices are made for I2C (serial computer bus) communication with a computer. Fortunately, the Jetson has that sort of access. It would also be nice if the IMU has level shifting, so that we don’t have to worry too much about voltages and such. Before we can get data out of the IMU, we’ll have to build and install some software. 
 
## Installation
I suggest you use the Richards-Tech repository if you have long term projects that need the library, as it is well maintained there and updated frequently. All you have to do is clone the appropriate repository, i.e. :
```
git clone https:://github.com/jetsonhacks/RTIMULib.git
```
Then go to the RTIMULib/Linux/RTIMULibDemoGL directory. There you will find the file RTIMULibDemoGL.pro, which is the Qt Creator project file that can be used to build the library and sample demonstration program. Here’s the blog post to Install Qt Creator on NVIDIA Jetson TK1.

As in the video, run Qt Creator on the RTIMULibDemoGL.pro file, ‘Build All’ and then run the program. Note that to access an IMU on I2C, the program must be run as super user, ‘sudo’ the program in other words from a Terminal. Also, it will need an IMU attached, but that’s another post! Click here for Part II.
I’ll show you what happens after you hook up the IMU to the Jetson, so that you know you’ve successfully hooked it up. Look here: https://www.youtube.com/watch?v=oTph-mUU-9g
## Background
For the IMU, the The Adafruit 10-DOF IMU Breakout is being used. We’ll go over hooking up the IMU to the Jetson in a later post, but it’s pretty easy. Bascially solder the supplied header onto the IMU, then wire the IMU header pins to the Jetson JA31 connector as follows:
```
GND J3A1-14 (-)
VCC J3A1-16 (+)
SCL J3A1-18 (C)
SDA J3A1-20 (D)
```
Here’s some pictures from the Jetson Wiki:
There are several ways to actually connect the wires to the pins, you can use headers, machine pin jumper wire, and a mix of simple Arduino jumpers connected to the appropriate header insert. We’ll look at this in a later blog entry.

## Calibration
Anyway, once the IMU is hooked up, the next step is to calibrate the IMU. If you have seen people rotating their quadcopter or model airplane and ‘posing’ them, this is what they are doing. There’s such a large amount of information that’s out there about the subject that there’s not much to add, other than you’ll have to calibrate the IMU. Basically the accelerometers need to be told the proper range of values when no artificial acceleration is being applied, and the magnetometers also need a little help. Here’s the RTIMULib calibration instructions, use them. Here’s why you want to calibrate the magnetometer.

## Conclusion
The IMU is very useful for finding out pose (orientation in space), and now they are almost ubiquitous in any type of sophisticated electronic or robotic device. It is one of the very useful tools in the electronic sensor toolbox. They are also lots of fun to play with. 
There are several ways to connect the Inertial Measurement Unit (IMU) to the Jetson TK1 Development Kit. Here’s a demonstration of a very simple way. Look here: https://www.youtube.com/watch?v=6SdbInw2ErY
## Background
For the IMU, the The Adafruit 10-DOF IMU Breakout is being used. Here’s a couple of pictures of the breakout board from the Adafruit website:
Solder the supplied header pins onto the IMU breakout board, then wire the IMU header pins to the Jetson J3A1 connector as follows:
```
GND J3A1-14 (-)
VCC J3A1-16 (+)
SCL J3A1-18 (C)
SDA J3A1-20 (D)
```
Unfortunately my soldering skills are so poor that providing a video of that process would probably be the canonical way *not* to solder. Note: Here’s a page with some information on how to get started soldering and working with electronic components if you need some help..

Here’s some pictures from the Jetson Wiki for reference :
There are several ways to actually connect the headers to the Jetson J3A1 connector, in this case we use simple female to female jumper wires (0.1″ spacing, 2.54mm pitch – this is standard Arduino size) to attach to the IMU header, and then connect the jumper wire to a machine pin jumper wire, which is then connected to the Jetson.

For the demo, I used Adafruit Premium Female/Female Jumper Wires – 40×6″ and 20 CM Machine Pin Wire Kit/10 Pack. This approach may be adequate for some projects, but for more rugged projects you may want to consider actually soldering wires to the header pins approach or making a breakout board for the Jetsons’ J3A1 header. Remember that the J3A1 header is 2mm pitch (0.08″) which is slightly smaller than the more standard DIY 2.54mm pitch that something like an Arduino uses. You’ll also want to physically mount the device some where also, fortunately there are mounting holes for that purpose on the breakout board.

Once you have the IMU connected to the Jetson, you’re ready to calibrate.
## Source
1. https://www.jetsonhacks.com/2015/04/22/inertial-measurement-unit-imu-part-i/
2. https://www.jetsonhacks.com/2015/04/23/inertial-measurement-unit-imu-part-ii/
3. https://www.jetsonhacks.com/2015/04/28/inertial-measurement-unit-imu-part-iii/
4. https://github.com/mrbichel/RTIMULib
