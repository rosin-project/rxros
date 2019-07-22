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
#include <brickpi3_msgs/Contact.h>
#include "Node.h"
#include "BrickPi3Touch.h"


BrickPi3Touch::BrickPi3Touch(const std::string& aName, const std::string& aFrameId, const std::string& aPort, const double aFrequency):
    touchPublisher(Node::getHandle().advertise<brickpi3_msgs::Contact>("/" + aName, 10))
{
    name = aName;
    frameId = aFrameId;
    port = port2id(aPort);
    freq = aFrequency;
    seqNo = 0;

    brickPi3.set_sensor_type(port, SENSOR_TYPE_TOUCH_NXT);
}


void BrickPi3Touch::schedulerCB()
{
    std::lock_guard<std::mutex> guard(mutex);

    sensor_touch_t sensorTouch;
    int rc = brickPi3.get_sensor(port, &sensorTouch);
    if (rc == 0) {
        ROS_DEBUG("Touch sensor: pressed %s", (sensorTouch.pressed) ? "True" : "False");

        brickpi3_msgs::Contact contact;
        contact.header.frame_id = frameId;
        contact.header.stamp = ros::Time::now();
        contact.header.seq = seqNo++;
        contact.contact = sensorTouch.pressed;

        touchPublisher.publish(contact);
    }
    else {
        ROS_ERROR("BrickPi3 failed to read Color sensor %s, rc %d", name.c_str(), rc);
    }
}
