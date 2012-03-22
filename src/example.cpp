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
 * Simple Example - it compiles, but doesn't do anything useful
 *
 ***********************************************/

#include "ros/ros.h"
#include <serial_node/serial.h>
#include <serial_node/MessageDB.hpp>

// handles all message traffic and could do other things too
class TestDB : public MessageDB {
public:
	void init(ros::NodeHandle n, std::string svc){
	    MessageDB::init(n,svc);
	}
	
	bool getData(){
	    std::string data; // in from arduino
	    std::string s = "<d>"; // out to arduino
	    
	    bool ok = getMessage(s,data);
	    
	    // handle error if ok = false
	    // parse data to get sensor data
	    
	    return ok;
	}
	
	// other useful functions
};

// a simple robot controller
class Robot {
public:
    Robot(ros::NodeHandle n, std::string svc){
        test_db.init(n,svc);
        
        // fill db with messages. this really depends on what you 
        // programmed your arduino to do!!!
        test_db.setMessage('a',6); // read analogs, expect 6 bytes in return
        test_db.setMessage('s',0); // start/stop robot, no data returned
        test_db.setMessage('d',34); // get data and return 34 bytes
        // more messages ...
        
    }
    
    void loop(int hz){
        ros::Rate r(hz);
        
        //Process ROS messages and send serial commands to uController
        while (ros::ok()){
            // do stuff
            test_db.getData();
            // do more stuff ...
            
            ros::spinOnce();
            r.sleep();
        }
    }
        
    // other robot functions ...
    
    
private:
    TestDB test_db;
};

// need to execute in another terminal:
//     "rosrun serial_port 0 /dev/cu.usbserial 9600 "  
//
int main(int argc, char **argv){
    //Initialize ROS
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle rosNode;
    
    // published name of serial service, note: the number could be
    // anything, uc4_serial for example, depends on how the serial node
    // was launched
    std::string svc = "uc0_serial"; 
    
    Robot robot(rosNode, svc);
    
    // run until user hits ^C to exit
    robot.loop(20);

    return 0;
}