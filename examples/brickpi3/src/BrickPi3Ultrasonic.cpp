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
#include <ros/ros.h>
#include <brickpi3_msgs/Range.h>
#include "Node.h"
#include "BrickPi3Ultrasonic.h"
using namespace std;


BrickPi3Ultrasonic::BrickPi3Ultrasonic(const std::string& aName, const std::string& aFrameId,const std::string& aPort, const double aFrequency, const double aMinRange, const double aMaxRange, const double aSpreadAngle):
    ultrasonicPublisher(Node::getHandle().advertise<brickpi3_msgs::Range>("/" + aName, 10))
{
    name = aName;
    frameId = aFrameId;
    port = port2id(aPort);
    freq = aFrequency;
    minRange = aMinRange;
    maxRange = aMaxRange;
    spreadAngle = aSpreadAngle;
    seqNo = 0;

    brickPi3.set_sensor_type(port, SENSOR_TYPE_NXT_ULTRASONIC);
}


void BrickPi3Ultrasonic::schedulerCB()
{
    sensor_ultrasonic_t sensorUltrasonic;
    int rc = brickPi3.get_sensor(port, &sensorUltrasonic);
    if (rc == 0) {
        ROS_DEBUG("Ultrasonic sensor: CM %5.1f Inches %5.1f", sensorUltrasonic.cm, sensorUltrasonic.inch);

        brickpi3_msgs::Range range;
        range.header.frame_id = frameId;
        range.header.stamp = ros::Time::now();
        range.header.seq = seqNo++;
        range.range = sensorUltrasonic.cm / 100.0;
        range.range_min = minRange;
        range.range_max = maxRange;
        range.spread_angle = spreadAngle;

        ultrasonicPublisher.publish(range);
    }
    else {
        ROS_ERROR("BrickPi3 failed to read Ultrasonic sensor %s, rc %d", name.c_str(), rc);
    }
}
