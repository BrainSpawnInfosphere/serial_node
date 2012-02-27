/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//r2Serial.cpp

// communicate via RS232 serial with a remote uController.
// communicate with ROS using String type messages.
// subscribe to command messages from ROS
// publish command responses to ROS

// program parameters - ucontroller# (0,1), serial port, baud rate

//Thread main
//  Subscribe to ROS String messages and send as commands to uController
//Thread receive
//  Wait for responses from uController and publish as a ROS messages


#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <sstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <queue> // C++ FIFO

#include "serial_node/serial.h"

//#define DEFAULT_BAUDRATE 19200
//#define DEFAULT_SERIALPORT "/dev/ttyUSB0"


class Serial {
public:
    Serial(ros::NodeHandle& n, int i=0){
        char sname[30];
        sprintf(sname, "uc%d_serial",i);
        name = sname;
        service = n.advertiseService(name, &Serial::callback, this);
    }
    
    ~Serial(){
        ::close(fd);
        ROS_INFO("%s stopping", name.c_str());
    }

    inline void flush(void){ //discard data that was not read
		tcflush (fd, TCIFLUSH);
	}
	
    bool callback(serial_node::serial::Request& req,
            serial_node::serial::Response& res);
    
    int read(char *buf, const int numbytes, const int trys=3);
    //int read2(char *buf, const int numbytes, const int trys=3);
    int write(const char* buf, const int numbytes, const int trys=3);
    bool open(char *port, int baud);
    unsigned int available();

//protected:
    ros::ServiceServer service;
    int fd;   //serial port file pointer
    std::string name;
};

//Initialize serial port, return file descriptor
bool Serial::open(char *port, int baud){
        char msg[70];
		struct termios options;
		
		fd = ::open( port, O_RDWR | O_NOCTTY | O_NDELAY /*| O_NONBLOCK*/ ); //may not need the nodelay
		if( fd <= 0 ){
			sprintf(msg,"ERROR: Unable to open port %s:%d - %s",port,baud,strerror(errno));
			perror(msg);
			//return(-1);
			//throw kException(msg);
			return false;
		}
		
		fcntl(fd, F_SETFL, FNDELAY); // return if nothing there to read
		
		//get config from fd and put into options
		tcgetattr (fd, &options); 
		
		// Enable the receiver and set local mode
		options.c_cflag |= (CLOCAL | CREAD);
		
		//give raw data path
		cfmakeraw (&options);
		
		//set baud
		cfsetspeed(&options, baud);
		
		//send options back to fd
		tcsetattr (fd, TCSANOW, &options);
		
		Serial::flush();
    
    return true;
} //serialInit

#define __K_SERIAL_DEBUG__ 0

/*
int Serial::read2(char *buf, const int numbytes, const int trys){
    int i, numread = 0, n = 0, numzeroes = 0;
    
    while (numread < numbytes){
        //printf("%d .\n", fd);
        n = ::pread (fd, buf, 1, numread);
        
        if (n < 0)
            return -1;
        else if (0 == n){
            numzeroes++;
            if (numzeroes > trys)
                break;
        }
        else {
            numread++;
            numzeroes = 0;
        
        }
        
    }
    
    tcflush (fd, TCIFLUSH);			//discard data that was not read
    return numread;
}*/

int Serial::read(char *buf, const int numbytes, const int trys){
    int i, numread = 0, n = 0, numzeroes = 0;
    
    while (numread < numbytes){
        printf("%d .\n", fd);
        n = ::read (fd, (buf + numread), (numbytes - numread));
        if (n < 0)
            return -1;
        else if (0 == n){
            numzeroes++;
            if (numzeroes > trys)
                break;
        }
        else numread += n;
    }
    
    if (__K_SERIAL_DEBUG__){
        printf ("Read:   ");
        for (i = 0; i < numread; i++)
            printf ("%d ", buf[i]);
        printf ("\nRead %d of %d bytes\n", numread, numbytes);
    }
    
    //tcflush (fd, TCIFLUSH);			//discard data that was not read
    return numread;
}


/**
 * Determine the number of bytes in the input buffer without the need to 
 * read it.
 */
unsigned int Serial::available(void){
    int bytes = 0;
    ioctl(fd, FIONREAD, &bytes);
    
    //ROS_INFO("bytes avail: %d",bytes);
    
    return (unsigned int)bytes;
}

/**
 * Write a specific number of bytes from a buffer
 * \note write() will attempt to read the serial port several
 *       times before failing
 * \return number of bytes written
 */
int Serial::write(const char* buf, const int numbytes, const int trys){
    int i, numwritten = 0, n = 0, numzeroes = 0;

    while (numwritten < numbytes){
        n = ::write (fd, (buf + numwritten), (numbytes - numwritten));
        
        if (n < 0) return -1;
        
        else if (0 == n){
            numzeroes++;
            
            if (numzeroes > trys) break;
        }
        else numwritten += n;
    }
    
    if (__K_SERIAL_DEBUG__){
        printf ("cwrite[%d]: ", numbytes);
        for (i = 0; i < numbytes; i++) printf ("%d ", buf[i]);
        printf ("\n");
    }
    
    return numwritten;
}

//Process ROS command message, send to uController
bool Serial::callback(serial_node::serial::Request& req,
            serial_node::serial::Response& res){
            
    //Serial::flush();
    
    //ROS_INFO("%s command: %s", name.c_str(), req.str.c_str());
    Serial::write(req.str.c_str(),req.str.size());
    
    
    //ROS_INFO("available: %d",available());
    /*
    if(req.size == 0){
        res.str = " ";
        return true;
    }
    */
    // wait x msec
    int miss = 0;
    while( available() < req.size ){
        if(miss++ < req.time) usleep(1000);
        else break;
        //ROS_INFO("loop: %d", available());
    }
    
    //ROS_INFO("going to read");
    int rcvBufSize = 200;
    char ucResponse[rcvBufSize];
    int num = (req.size > available() ? available() : req.size);
    int n = Serial::read(ucResponse,num);
    //ROS_INFO("done to read");
    
    
    Serial::flush();
    
    if (n > 0) { 
        ROS_INFO("%s response: %s", name.c_str(), ucResponse);
        res.str.assign(ucResponse,n);
    }
    else /*if ( n == -1 || n == 0)*/{
        ROS_ERROR("Read Error");
        res.str = " ";
        return false;
    }
    //else return false;
    
    return true;
        
} //ucCommandCallback


int main(int argc, char **argv)
{
    char *port;    //port name
    int baud;     //baud rate
    int uC;

    //Initialize ROS
    ros::init(argc, argv, "r2SerialDriver");
    ros::NodeHandle rosNode;
    
    Serial serial(rosNode,0);

    //Open and initialize the serial port to the uController
    if (argc > 1) uC = atoi(argv[1]);
    else {
        ROS_ERROR("ucontroller index parameter invalid");
        return 1;
    }
    

    //strcpy(port, DEFAULT_SERIALPORT);
    if (argc > 2) port = argv[2];
    else { 
        ROS_ERROR("need port defined");
        return 1;
    }

    if (argc > 3) baud = atoi(argv[3]);
    else {
            ROS_ERROR("ucontroller baud rate parameter invalid");
            return 1;
    }

    //ROS_INFO("Service %s on %s @ %d baud",serial.name.c_str(), port, baud);
    
    if (serial.open(port, baud) == false){
        ROS_ERROR("unable to create a new serial port");
        return 1;
    }
    else ROS_INFO("Service %s on %s @ %d baud",serial.name.c_str(), port, baud);

    //Process ROS messages and send serial commands to uController
    ros::spin();

    return 0;
}