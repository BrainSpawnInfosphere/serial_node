# ROS Node: Serial_Node

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko

Simple node that connects to a serial port and launches a ROS service. Ideally I would use rosserial, but that doesn't seem to work reliably and there appear (at this time) to be any real progress done on it in the last several months.

## Command Line

	rosrun serial_port serial_port <uC> <port> <baud>

* uC: a number that gets appended to the service topic incase there are multiple nodes running at once … 0, 1, 2, 3, …
* port: the serial port
* baud: the baud rate to connect at: 9600, …, 57600, 115200

### Published Services: 
**string:** uc<uC>_serial where <uC> was passed as a command line parameter
 
### Example:
 	rosrun serial_port serial_port 0 /dev/cu.usbserial 9600

## To Do

* it basically works, but I am still trying to think how best to do it
