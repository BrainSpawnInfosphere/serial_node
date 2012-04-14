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

/***********************************************
 *
 * rosrun serial_node <uC> <port> <baud>
 *
 * uC   = 0, 1, 2, ... what number micro controller 
 * port = serial port, /dev/cu.usbserial
 * baud = baud rate, 115200
 *
 ***********************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
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
#include <iostream>

#include "serial_node/serial.h"


class Serial {
public:
    Serial(ros::NodeHandle& n, int i=0){
        char sname[30];
        sprintf(sname, "uc%d_serial",i);
        name = sname;
        service = n.advertiseService(name, &Serial::callback, this);
        debug = false;
        read_error = read_good = 0;
    }
    
    ~Serial(){
        ::close(fd);
        ROS_INFO("%s stopping", name.c_str());
    }
    
    inline void setDebug(bool b=true){ debug = b; }

    inline void flush(void){ //discard data that was not read
		tcflush (fd, TCIFLUSH);
	}
	
    bool callback(serial_node::serial::Request& req,
            serial_node::serial::Response& res);
    
    int read(char *buf, const int numbytes, const int trys=3);
    bool readMsg(std::string&, int size, const int trys=3);
    //int read2(char *buf, const int numbytes, const int trys=3);
    int write(const char* buf, const int numbytes, const int trys=3);
    bool open(char *port, int baud);
    unsigned int available();

protected:
    unsigned short checksum(char* buffer, int cnt);
    ros::ServiceServer service;
    int fd;   //serial port file pointer
    std::string name;
    bool debug;
    unsigned long read_error;
    unsigned long read_good;
};

// check this!!
unsigned short Serial::checksum(char* buffer, int cnt){
    unsigned int sum = 0; // 32 bit int
    
    for(int i=0;i<cnt;i+=2) sum += *((unsigned short*)buffer++);
    sum = (sum & 0xFFFF) + (sum >> 16);
    
    return(~sum);
}

//Initialize serial port, return file descriptor
bool Serial::open(char *port, int baud){
        char msg[70];
		struct termios options;
		
		fd = ::open( port, O_RDWR | O_NOCTTY | O_NDELAY /*| O_NONBLOCK*/ ); //may not need the nodelay
		if( fd <= 0 ){
		    // [FIXME 20120322] errno not defined in linux
			sprintf(msg,"ERROR: Unable to open port %s:%d - %s",port,baud,strerror(errno));
			perror(msg);
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

int Serial::read(char *buf, const int numbytes, const int trys){
    int i, numread = 0, n = 0, numzeroes = 0;
    
    while (numread < numbytes){
        //printf("%d .\n", fd);
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

bool getChar(char& c, int fd, const int trys=20){
    int n = 0;
    int numzeroes = 0;
    
    // wait until start char found
    while (numzeroes++ < trys){
        n = ::read (fd, &c, 1);
        if(n == 1){
            return true;
        }
        usleep(1000);
    }
    
    return false;
}

inline char makeReadable(char c){
    return (c < 32 ? '.' : (c > 126 ? '.' : c));
}

void printMsg(std::string& msg){
    std::cout<<"---[[ "<<msg[1]<<" ]]-----------------------------------"<<std::endl;
    std::cout<<"Size data: "<<(int)msg[2]<<"\t"<<"Size Msg: "<<msg.size()<<std::endl;
    std::cout<<"Start/End characters: "<<msg[0]<<" / "<<msg[msg.size()-1]<<std::endl;
    int size = (int)msg[2];
    if(size > 0){
        std::cout<<"Raw:  ";
        for(int i=0;i<size;i++) std::cout<<(int)msg[3+i]<<' ';
        std::cout<<std::endl;
        std::cout<<"ASCII: ";
        for(int i=0;i<size;i++) std::cout<<makeReadable(msg[3+i])<<' ';
        std::cout<<std::endl;
    }
    std::cout<<"-------------------------------------------"<<std::endl;
}

bool Serial::readMsg(std::string& ucString, int size, const int trys){
    int numread = 0, n = 0, numzeroes = 0;
    char c = 0;
    char buf[128];
    
    // wait until start char found
    while (numzeroes++ < trys){
        n = ::read (fd, &c, 1);
        if(c == '<') break;
        usleep(1000);
    }
    
    // get start char
    while(c != '<'){
        if(!getChar(c,fd)) return false;
    }
    buf[0] = c;
    numread++;
    //ROS_INFO("got start char");
    
    // get message char
    if(!getChar(c,fd)) return false;
    buf[1] = c;
    numread++;
    
    //get message size
    if(!getChar(c,fd)) return false;
    buf[2] = c;
    numread++;
    n = (int)c;
    //ROS_INFO("data size: %d",n);
    
    // get data section
    if(n){
        for(int i=0;i<n;i++){
            if(!getChar(c,fd)) return false;
            buf[3+i] = c;
            numread++;
        }
    }
    
    // get end char
    if(!getChar(c,fd)) return false;
    if(c != '>') return false;
    buf[numread] = c;
    numread++;
    //ROS_INFO("got end char");
    
    ucString.assign(buf,numread);
    
    if(debug) printMsg(ucString);
    
    return true;
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
 * \note write() will attempt to write to the serial port several
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
            
    Serial::flush();
    
    //ROS_INFO("%s command: %s", name.c_str(), req.str.c_str());
    Serial::write(req.str.c_str(),req.str.size());
    //usleep(1000);
    
    //ROS_INFO("send: %s[%d]",req.str.c_str(),req.str.size());
    
    // wait x msec
    int miss = 0;
    while( available() < req.size ){
        if(miss++ < req.time) usleep(1000);
        else { // resend
            //Serial::flush();
            Serial::write(req.str.c_str(),req.str.size());
            miss = 0;
            ROS_INFO("Resending: avail[%d] need[%d]",
                Serial::available(), req.size);
            // QoS
            ++read_error;
        }
        
        //if(!ros::ok()) return 0;
        //ROS_INFO("loop: %d", available());
    }
    
    
    if(req.size > 0){
        usleep(1000);
        
        //ROS_INFO("going to read");
        //int rcvBufSize = 200;
        //char ucResponse[rcvBufSize];
        std::string ucString;
        //int num = (req.size > available() ? available() : req.size);
        bool ok = Serial::readMsg(ucString,req.time);
        //ROS_INFO("got:%s",ucResponse);
        //ROS_INFO("done to read");
              
        if (ok) { 
            //ROS_INFO("%s response: %s", name.c_str(), ucResponse);
            res.str = ucString;
            
            // QoS
            ++read_good;
        }
        else{
            ROS_ERROR("Error: %s",req.str.c_str());
            res.str = "error";
            Serial::flush();
            
            // QoS
            ++read_error;
            return false;
        }
    }
    else res.str = "success";
    
    
    Serial::flush();
    return true;
        
} //ucCommandCallback



int main(int argc, char **argv){
    char *port;    //port name
    int baud;     //baud rate
    int uC;

    //Initialize ROS
    ros::init(argc, argv, "serial_node");
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
    
    bool debug;
    if (argc > 4) debug = (strcmp(argv[4],"true") ? false : true);
    else debug = false;
    serial.setDebug(debug);

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