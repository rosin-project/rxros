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

#include <mutex>
#include <string>
#include <stdio.h>
#include <Scheduler.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <teleop_msgs/Joystick.h>
#include <teleop_msgs/Keyboard.h>
#include <geometry_msgs/Twist.h>
#include "JoystickPublisher.h"
#include "KeyboardPublisher.h"


class VelocityPublisher {
private:
    std::mutex mutex;
    Bosma::Scheduler scheduler;
    ros::NodeHandle nodeHandle;
    ros::Publisher cmdVelPublisher;
    ros::Subscriber joystickSubscriber;
    ros::Subscriber keyboardSubscriber;
    int publishEveryMs;
    double minVelLinear;
    double maxVelLinear;
    double minVelAngular;
    double maxVelAngular;
    double deltaVelLinear;
    double deltaVelAngular;
    double currVelLinear;
    double currVelAngular;

    // Callback functions for ROS topics.
    void schedulerCB();
    void joystickCB(const teleop_msgs::Joystick& joy);
    void keyboardCB(const teleop_msgs::Keyboard& key);

public:
    VelocityPublisher(int argc, char** argv);
    virtual ~VelocityPublisher() {};
    void run() {ros::spin();}
};

VelocityPublisher::VelocityPublisher(int argc, char** argv) :
    cmdVelPublisher(nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
    joystickSubscriber(nodeHandle.subscribe("/joystick", 10, &VelocityPublisher::joystickCB, this)),
    keyboardSubscriber(nodeHandle.subscribe("/keyboard", 10, &VelocityPublisher::keyboardCB, this))
{
    nodeHandle.param("/velocity_publisher/publish_every_ms", publishEveryMs, 100);
    nodeHandle.param("/velocity_publisher/min_vel_linear", minVelLinear, 0.04); // m/s
    nodeHandle.param("/velocity_publisher/max_vel_linear", maxVelLinear, 0.10); // m/s
    nodeHandle.param("/velocity_publisher/min_vel_angular", minVelAngular, 0.64); // rad/s
    nodeHandle.param("/velocity_publisher/max_vel_angular", maxVelAngular, 1.60); // rad/s
    deltaVelLinear = (maxVelLinear - minVelLinear) / 10.0;
    deltaVelAngular = (maxVelAngular - minVelAngular) / 10.0;
    currVelLinear = 0.0;
    currVelAngular = 0.0;

    printf("publish_every_ms: %d\n", publishEveryMs);
    printf("min_vel_linear: %f m/s\n", minVelLinear);
    printf("max_vel_linear: %f m/s\n", maxVelLinear);
    printf("min_vel_angular: %f rad/s\n", minVelAngular);
    printf("max_vel_angular: %f rad/s\n", maxVelAngular);

    scheduler.every(std::chrono::milliseconds(publishEveryMs), boost::bind(&VelocityPublisher::schedulerCB, this));
}


void VelocityPublisher::schedulerCB()
{
    std::lock_guard<std::mutex> guard(mutex);

    geometry_msgs::Twist vel;
    vel.linear.x = currVelLinear;
    vel.angular.z = currVelAngular;
    cmdVelPublisher.publish(vel);
}


void VelocityPublisher::joystickCB(const teleop_msgs::Joystick& joy)
{
    std::lock_guard<std::mutex> guard(mutex);

    JoystickEvents event = static_cast<JoystickEvents>(joy.event);
    switch (event) {
        case JS_EVENT_BUTTON0_DOWN:
            currVelLinear = 0.0;
            currVelAngular = 0.0;
            break;
        case JS_EVENT_BUTTON1_DOWN:
            currVelLinear = 0.0;
            currVelAngular = 0.0;
            break;
        case JS_EVENT_AXIS_UP:
            currVelLinear += deltaVelLinear;
            if (currVelLinear > maxVelLinear) {
                currVelLinear = maxVelLinear;
            }
            else if (currVelLinear > -minVelLinear && currVelLinear < minVelLinear) {
                currVelLinear = minVelLinear;
            }
            break;
        case JS_EVENT_AXIS_DOWN:
            currVelLinear -= deltaVelLinear;
            if (currVelLinear < -maxVelLinear) {
                currVelLinear = -maxVelLinear;
            }
            else if (currVelLinear > -minVelLinear && currVelLinear < minVelLinear) {
                currVelLinear = -minVelLinear;
            }
            break;
        case JS_EVENT_AXIS_LEFT:
            currVelAngular -= deltaVelAngular;
            if (currVelAngular < -maxVelAngular) {
                currVelAngular = -maxVelAngular;
            }
            else if (currVelAngular > -minVelAngular && currVelAngular < minVelAngular) {
                currVelAngular = -minVelAngular;
            }
            break;
        case JS_EVENT_AXIS_RIGHT:
            currVelAngular += deltaVelAngular;
            if (currVelAngular > maxVelAngular) {
                currVelAngular = maxVelAngular;
            }
            else if (currVelAngular > -minVelAngular && currVelAngular < minVelAngular) {
                currVelAngular = minVelAngular;
            }
            break;
        default:
            break;
    }
}


void VelocityPublisher::keyboardCB(const teleop_msgs::Keyboard& key)
{
    std::lock_guard<std::mutex> guard(mutex);

    KeyboardEvents event = static_cast<KeyboardEvents>(key.event);
    switch (event) {
        case KB_EVENT_UP:
            currVelLinear += deltaVelLinear;
            if (currVelLinear > maxVelLinear) {
                currVelLinear = maxVelLinear;
            }
            else if (currVelLinear > -minVelLinear && currVelLinear < minVelLinear) {
                currVelLinear = minVelLinear;
            }
            break;
        case KB_EVENT_LEFT:
            currVelAngular -= deltaVelAngular;
            if (currVelAngular < -maxVelAngular) {
                currVelAngular = -maxVelAngular;
            }
            else if (currVelAngular > -minVelAngular && currVelAngular < minVelAngular) {
                currVelAngular = -minVelAngular;
            }
            break;
        case KB_EVENT_RIGHT:
            currVelAngular += deltaVelAngular;
            if (currVelAngular > maxVelAngular) {
                currVelAngular = maxVelAngular;
            }
            else if (currVelAngular > -minVelAngular && currVelAngular < minVelAngular) {
                currVelAngular = minVelAngular;
            }
            break;
        case KB_EVENT_DOWN:
            currVelLinear -= deltaVelLinear;
            if (currVelLinear < -maxVelLinear) {
                currVelLinear = -maxVelLinear;
            }
            else if (currVelLinear > -minVelLinear && currVelLinear < minVelLinear) {
                currVelLinear = -minVelLinear;
            }
            break;
        case KB_EVENT_SPACE:
            currVelLinear = 0.0;
            currVelAngular = 0.0;
            break;
        default:
            break;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_publisher"); // Name of this node.
    VelocityPublisher velocityPublisher(argc, argv);
    velocityPublisher.run();
}
