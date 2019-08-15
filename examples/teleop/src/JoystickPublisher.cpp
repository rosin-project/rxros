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

#include <fcntl.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <teleop_msgs/Joystick.h>
#include "JoystickPublisher.h"

#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */

struct joystick_event {
    unsigned int time;      /* event timestamp in milliseconds */
    short value;            /* value */
    unsigned char type;     /* event type */
    unsigned char number;   /* axis/button number */
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_publisher"); // Name of this Node.
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<teleop_msgs::Joystick>("/joystick", 10); // Publisher Topic /joystick

    // Read parameter device
    std::string joystickDevice;
    nodeHandle.param<std::string>("/joystick_publisher/device", joystickDevice, "/dev/input/js0");

    int fd = open(joystickDevice.c_str(), O_RDONLY);
    if( fd < 0 ) {
        printf("Cannot open joystick device.\n");
        exit(1);
    }

    struct joystick_event joystickEvent;
    while (ros::ok()) {
        read(fd, &joystickEvent, sizeof(joystickEvent));
        if (joystickEvent.type == JS_EVENT_BUTTON || joystickEvent.type == JS_EVENT_AXIS) {
            ros::Time rosTime(joystickEvent.time, 0);
            teleop_msgs::Joystick joystick;
            joystick.time = rosTime;

            unsigned char type = joystickEvent.type;
            unsigned char number = joystickEvent.number;
            short value = joystickEvent.value;
            joystick.event = JS_EVENT_NEUTRAL;
            if (type == JS_EVENT_BUTTON) {
                if (number == 0 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON0_UP;
                }
                else if (number == 0 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON0_DOWN;
                }
                else if (number == 1 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON1_UP;
                }
                else if (number == 1 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON1_DOWN;
                }
                else if (number == 2 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON2_UP;
                }
                else if (number == 2 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON2_DOWN;
                }
                else if (number == 3 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON3_UP;
                }
                else if (number == 3 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON3_DOWN;
                }
            }
            else if (type == JS_EVENT_AXIS) {
                if (number==0 && value == 32767)
                {
                    joystick.event = JS_EVENT_AXIS_RIGHT;
                }
                else if (number==0 && value==-32767)
                {
                    joystick.event = JS_EVENT_AXIS_LEFT;
                }
                else if (number==1 && value==32767)
                {
                    joystick.event = JS_EVENT_AXIS_DOWN;
                }
                else if (number==1 && value==-32767) {
                    joystick.event = JS_EVENT_AXIS_UP;
                }
            }

            if (joystick.event != JS_EVENT_NEUTRAL) {
                publisher.publish(joystick);
            }
        }
    }
}
