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

#ifndef BRICKPI3_BRICKPI3TOUCH_H
#define BRICKPI3_BRICKPI3TOUCH_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include "BrickPi3Device.h"


class BrickPi3Touch: public BrickPi3Device
{
private:
    ros::Publisher touchPublisher;
    std::string name;
    std::string frameId;
    uint8_t port;
    double freq;
    unsigned int seqNo;

public:
    BrickPi3Touch(const std::string& name, const std::string& frameId, const std::string& port, const double frequency);
    virtual ~BrickPi3Touch() {}

    const std::string& getName() const {return name;}
    const std::string& getFrameId() const {return frameId;}
    const uint8_t getPort() const {return port;}
    const double getFreq() const {return freq;}

    void schedulerCB();
};


#endif //BRICKPI3_BRICKPI3TOUCH_H
