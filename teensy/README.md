## Setup 
Setup rosserial to communicate with Teensy via ROS. It also involves command for generation of custom messages headers which can be accessed by code in arduino.

### Steps to setup Arduino with Teensy:
1. Setup arduino with Teensy package using [https://www.pjrc.com/teensy/td_download.html](https://www.pjrc.com/teensy/td_download.html)
2. Setup rosserial:
    - [http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
	- run:
	    - `sudo apt-get install ros-indigo-rosserial-arduino`
	    - `sudo apt-get install ros-indigo-rosserial`
3. To setup libraries:
	1. First source all your workspace to make sure you don't miss out in generating message headers for your package.
	2. Go to the libraries folder inside arduino sketch book. 
    3. Run:
	    - `cd <sketchbook>/libraries`
	    - `rm -rf ros_lib`
	    - `rosrun rosserial_arduino make_libraries.py .`
4. Once this is configured: 
    1. Choose Teensy 3.2/3.1 in Tools menu of arduino software. Also make sure settings are configured to : "serial" and "96 Mhz optimized".
    2. After choosing the appropriate port, your arduino should be ready to dump the firmware.
