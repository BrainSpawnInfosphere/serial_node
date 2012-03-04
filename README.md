# ROS Node: Serial_Node

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko

Simple node that connects to a serial port and launches a ROS service. Ideally I would use rosserial, but that doesn't seem to work reliably and there appear (at this time) to be any real progress done on it in the last several months.

This node looks for a simple message format:

   < message size data >

where
* < is the start of a message
* message is one char (e.g., m, a, c, etc) that defines the type of message
* size is one byte that describes how much data follows, can be 0-255
* data is the message data
* > is the end of a message

At this time I do not do any kind of check sum, but I am thinking about it.

## Command Line

	rosrun serial_port serial_port <uC> <port> <baud> <debug>

* uC: a number that gets appended to the service topic incase there are multiple nodes running at once … 0, 1, 2, 3, …
* port: the serial port
* baud: the baud rate to connect at: 9600, …, 57600, 115200
* debug: enable debugging info printing ... just put true if you want it

## Published Services: 
The service message format is:
* request str: command sent to robot
* request size: size of returned message from robot, tells serial_node how much data to look for
* request time: how long (in msec) to wait for a response before declaring failure
* response str: the data sent back from the robot

The service is advertized as uc<uC>_serial where <uC> is the number you passed on the command line

### Example:
 	rosrun serial_port serial_port 0 /dev/cu.usbserial 9600 true

## To Do

* it basically works, but I am still trying to think how best to do it
