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

#ifndef BRICKPI3_BRICKPI3DEVICE_H
#define BRICKPI3_BRICKPI3DEVICE_H

#include <climits>
#include <string>
#include <mutex>
#include "BrickPi3.h"


static uint8_t port2id(const std::string& port)
{
    int id = ((port == "PORT_A") ? PORT_A :
             ((port == "PORT_B") ? PORT_B :
             ((port == "PORT_C") ? PORT_C :
             ((port == "PORT_D") ? PORT_D :
             ((port == "PORT_1") ? PORT_1 :
             ((port == "PORT_2") ? PORT_2 :
             ((port == "PORT_3") ? PORT_3 :
             ((port == "PORT_4") ? PORT_4 : INT_MAX))))))));
    return static_cast<uint8_t>(id);
}


class BrickPi3Device
{
protected:
    BrickPi3 brickPi3;
    std::mutex mutex;

public:
    BrickPi3Device() {brickPi3.detect();}
    virtual ~BrickPi3Device() {brickPi3.reset_all();}

    virtual void schedulerCB() = 0;
};

#endif //BRICKPI3_BRICKPI3DEVICE_H
