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
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <brickpi3_msgs/JointCommand.h>
#include "Node.h"
#include "BrickPi3Device.h"
#include "BrickPi3Motor.h"
using namespace std;

#define POWER_MAX 1.0


BrickPi3Motor::BrickPi3Motor(const string& aName, const string& aPort, const double aFrequency):
    jointStatePublisher(Node::getHandle().advertise<sensor_msgs::JointState>("/joint_state", 10)),
    jointCommandSubscriber(Node::getHandle().subscribe("/joint_command", 10, &BrickPi3Motor::jointCommandCB, this)),
    lastTime(0,0),
    lastPosition(0)
{
    name = aName;
    port = port2id(aPort);
    freq = aFrequency;
    seqNo = 0;

    brickPi3.reset_motor_encoder(port);
}

BrickPi3Motor::~BrickPi3Motor()
{
    brickPi3.reset_motor_encoder(port);
}

void BrickPi3Motor::jointCommandCB(const brickpi3_msgs::JointCommand& jointCommand)
{
    lock_guard<std::mutex> guard(mutex);

    if (name == jointCommand.name) {
        //Store the commanded power value, limited to the range +/- POWER_MAX

        effort = jointCommand.effort;
        if (effort > POWER_MAX) {
            effort = POWER_MAX;
        } else if (effort < -POWER_MAX) {
            effort = -POWER_MAX;
        }
    }
}

void BrickPi3Motor::schedulerCB()
{
    lock_guard<std::mutex> guard(mutex);

    uint8_t motorState = 0;// Variable for reading motor motorState
    int8_t motorPower = 0;// Variable for reading motor powers
    int32_t motorPosition = 0;  // Variable for reading motor encoder positions
    int16_t motorDPS = 0; // Variable for reading motor speeds (Degrees Per Second)

    int rc = brickPi3.get_motor_status(PORT_A, motorState, motorPower, motorPosition, motorDPS);
    if (rc == 0) {
        ROS_DEBUG("State A: %d Power A: %4d  Encoder A: %6d  DPS A: %6d", motorState, motorPower, motorPosition, motorDPS);

        ros::Time currTime = ros::Time::now();
        double currPosition = static_cast<double>(motorPosition) * M_PI / 180.0 / 3.0;
        double currVelocity = 0.0;
        if (!lastTime.isZero()) {
            currVelocity = (currPosition - lastPosition)/(currTime - lastTime).toSec();
        }
        lastPosition = currPosition;
        lastTime = currTime;

        sensor_msgs::JointState jointState;
        //jointState.header.frame_id = name;
        jointState.header.stamp = ros::Time::now();
        jointState.header.seq = seqNo++;
        jointState.name.push_back(name);
        jointState.effort.push_back(effort);
        jointState.position.push_back(currPosition); // rad
        jointState.velocity.push_back(currVelocity); // rad/s

        jointStatePublisher.publish(jointState);

        brickPi3.set_motor_power(port, static_cast<int8_t>(effort * 100.0));
    }
    else {
        ROS_ERROR("BrickPi3 failed to read Motor status %s, rc %d", name.c_str(), rc);
    }
}
