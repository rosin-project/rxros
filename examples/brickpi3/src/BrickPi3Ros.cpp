/*
Copyright (c) 2019, ROSIN-project
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include <iostream>
#include <Scheduler.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include "Node.h"
#include "BrickPi3Motor.h"
#include "BrickPi3Ultrasonic.h"
#include "BrickPi3Color.h"
#include "BrickPi3Touch.h"

class BrickPi3Ros {
private:
    Bosma::Scheduler scheduler;

public:
    BrickPi3Ros(int argc, char** argv);
    virtual ~BrickPi3Ros() {}
    void run() {ros::spin();}
};

BrickPi3Ros::BrickPi3Ros(int argc, char** argv):
    scheduler(8) // BrickPi3 supports 8 devices.
{
    // Parse the ros_robot.yaml file and create the appropriate sensors and actuators
    XmlRpc::XmlRpcValue brickpi3_robot;
    Node::getHandle().getParam("/brickpi3/brickpi3_robot", brickpi3_robot);
    ROS_ASSERT(brickpi3_robot.getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::cout << "---------------------------------" << std::endl;
    for (int i = 0; i < brickpi3_robot.size(); i++) {
        XmlRpc::XmlRpcValue brickpi3_device = brickpi3_robot[i];
        std::string type = brickpi3_device["type"];
        if (type == "motor") {
            std::string name = brickpi3_device["name"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];
            std::cout << type << ", " << name << ", " << port << ", " << freq << std::endl;

            BrickPi3Motor* motor = new BrickPi3Motor(name, port, freq); //todo: Needs better resource handling. Should be freed when the BrickPi3Ros object is deleted.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Motor::schedulerCB, motor));
        }
        else if (type == "ultrasonic") {
            std::string name = brickpi3_device["name"];
            std::string frame_id = brickpi3_device["frame_id"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];
            double min_range = brickpi3_device["min_range"];
            double max_range = brickpi3_device["max_range"];
            double spread_angle = brickpi3_device["spread_angle"];

            BrickPi3Ultrasonic* ultrasonic = new BrickPi3Ultrasonic(name, frame_id, port, freq, min_range, max_range, spread_angle); //todo: Needs better resource handling.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Ultrasonic::schedulerCB, ultrasonic));
        }
        else if (type ==  "color")
        {
            std::string name = brickpi3_device["name"];
            std::string frame_id = brickpi3_device["frame_id"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];

            BrickPi3Color* color = new BrickPi3Color(name, frame_id, port, freq); //todo: Needs better resource handling.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Color::schedulerCB, color));
        }
        else if (type == "touch")
        {
            std::string name = brickpi3_device["name"];
            std::string frame_id = brickpi3_device["frame_id"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];

            BrickPi3Touch* touch = new BrickPi3Touch(name, frame_id, port, freq); //todo: Needs better resource handling.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Touch::schedulerCB, touch));
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "brickpi3"); // Name of this node.
    BrickPi3Ros brickPi3Ros(argc, argv);
    brickPi3Ros.run();
}
