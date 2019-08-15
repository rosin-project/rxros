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

#include <rxros.h>
#include <teleop_msgs/Joystick.h>
#include "JoystickPublisher.h"
using namespace rxcpp::operators;
using namespace rxros::operators;


#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */

struct joystick_event
{
    unsigned int time;      /* event timestamp in milliseconds */
    short value;            /* value */
    unsigned char type;     /* event type */
    unsigned char number;   /* axis/button number */
};

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "joystick_publisher"); // Name of this Node.

    const auto joystickDevice = rxros::parameter::get("/joystick_publisher/device", "/dev/input/js0");
    rxros::logging().info() << "Joystick device: " << joystickDevice;

    auto joystickEvent2JoystickMsg = [=](const auto joystickEvent) {
        auto makeJoystickMsg = [=] (auto event) {
            teleop_msgs::Joystick joystickMsg;
            joystickMsg.time = ros::Time(joystickEvent.time, 0);
            joystickMsg.event = event;
            return joystickMsg;};
        if (joystickEvent.type == JS_EVENT_BUTTON) {
            if (joystickEvent.number == 0 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON0_UP);
            else if (joystickEvent.number == 0 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON0_DOWN);
            else if (joystickEvent.number == 1 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON1_UP);
            else if (joystickEvent.number == 1 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON1_DOWN);
            else if (joystickEvent.number == 2 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON2_UP);
            else if (joystickEvent.number == 2 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON2_DOWN);
            else if (joystickEvent.number == 3 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON3_UP);
            else if (joystickEvent.number == 3 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON3_DOWN);
        }
        else if (joystickEvent.type == JS_EVENT_AXIS) {
            if (joystickEvent.number == 0 && joystickEvent.value == 32767)
                return makeJoystickMsg(JS_EVENT_AXIS_RIGHT);
            else if (joystickEvent.number == 0 && joystickEvent.value == -32767)
                return makeJoystickMsg(JS_EVENT_AXIS_LEFT);
            else if (joystickEvent.number == 1 && joystickEvent.value == 32767)
                return makeJoystickMsg(JS_EVENT_AXIS_DOWN);
            else if (joystickEvent.number == 1 && joystickEvent.value == -32767)
                return makeJoystickMsg(JS_EVENT_AXIS_UP);
        }
        else
            return makeJoystickMsg(JS_EVENT_NEUTRAL);};

    rxros::observable::from_device<joystick_event>(joystickDevice)
        | map(joystickEvent2JoystickMsg)
        | publish_to_topic<teleop_msgs::Joystick>("/joystick");

    rxros::logging().info() << "Spinning joystick_publisher ...";
    rxros::spin();
}
