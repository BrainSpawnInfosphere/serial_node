/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 6 Mar 2011
 *********************************************************************
 * Status
 *  - Move to serial node ... this is robot agnostic
 *
 * Change Log:
 *  6 Mar 2011 Created
 *
 **********************************************************************
 *
 * 
 *
 */

#ifndef __MESSAGE_DATABASE_H__
#define __MESSAGE_DATABASE_H__

#include <ros/ros.h>
#include <std_msgs/String.h> // for simulation
#include "serial_node/serial.h"

#include <iostream>
#include <string> 


/**
 * This is a simple db that establishes a service connection to 
 * serial_node and sends/receives strings back and forth.
 */
class MessageDB {
public:
    // Constructor ... does nothing
    MessageDB(){
        ;
    }
    
    // sets up the client service to a serial node
    // n - node handle created in main()
    // svc - name of serial_node to connect too
    void init(ros::NodeHandle n, std::string svc){
		client = n.serviceClient<serial_node::serial>(svc);
	}
    
    // Send a message but get no response back
    inline bool getMessage(std::string& str){
        std::string ans;
        return getMessage(str,ans);
    }
    
    // Send a message and get a response back
    // str - message out
    // ans - reply back
    bool getMessage(std::string& str, std::string& ans){
        
        //ROS_INFO("sending: %s",str.c_str());
        
        // get the message character
        char msg = str[1];
        
        int size = 0;
        
        // did we find the message and get its size?
        if(!getMessageSize(msg,size)) return false;
        //ROS_INFO("mdb: found msg size[%d]",size);
        
	    serial_node::serial srv;
        srv.request.str = str;
        srv.request.size = size;
        srv.request.time = 100; // how do i do this?
        
        // did we get a response?
        //if(!client.call(srv)) return false; // what does this return?
        client.call(srv);
        //ROS_INFO("mdb: call worked");
        //ROS_INFO("mdb: resp: %s",srv.response.str.c_str());
        
        // if expecting data return ... copy string
        if(size > 0) ans = srv.response.str;
        
        return true;
    }
    
    /**
     * Add message to database
     * m - new message char
     * size - size of returned message back: 0-255
     */
    void setMessage(char m, int size){
        messages[m] = size;
    }

protected:    

    /**
     * Returns the message size or if message is not found, returns false
     */
    bool getMessageSize(char m, int& size){
        it = messages.find(m);
        
        if(it == messages.end()){
            size = 0;
            ROS_ERROR("Msg not found in database");
            return false;
        }
        
        size = it->second;
        
        return true;
    }

    std::map<char,int> messages;  // database of valid message sizes
    std::map<char,int>::iterator it; // database iterator
    
	ros::ServiceClient client; // ROS client service
};


#endif