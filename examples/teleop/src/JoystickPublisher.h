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

#ifndef TELEOP_JOYSTICKPUBLISHER_H
#define TELEOP_JOYSTICKPUBLISHER_H

enum JoystickEvents
{
    JS_EVENT_NEUTRAL = 0, // to avoid conflicts with KeyboardEvents
    JS_EVENT_BUTTON0_DOWN,
    JS_EVENT_BUTTON0_UP,
    JS_EVENT_BUTTON1_DOWN,
    JS_EVENT_BUTTON1_UP,
    JS_EVENT_BUTTON2_DOWN,
    JS_EVENT_BUTTON2_UP,
    JS_EVENT_BUTTON3_DOWN,
    JS_EVENT_BUTTON3_UP,
    JS_EVENT_AXIS_UP,
    JS_EVENT_AXIS_LEFT,
    JS_EVENT_AXIS_RIGHT,
    JS_EVENT_AXIS_DOWN
};

#endif //TELEOP_JOYSTICKPUBLISHER_H
