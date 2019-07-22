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

#ifndef RXCPP_LANG_BRICKPI3UTILS_H
#define RXCPP_LANG_BRICKPI3UTILS_H

#include <ros/ros.h>

// Support class to simplify access to configuration parameters for device
class DeviceConfig
{
private:
    std::string type;
    std::string name;
    std::string frameId;
    std::string port;
    double frequency;
    double minRange;
    double maxRange;
    double spreadAngle;

public:
    explicit DeviceConfig(XmlRpc::XmlRpcValue& value) {
        type = value.hasMember("type") ? (std::string) value["type"] : std::string("");
        name = value.hasMember("name") ? (std::string) value["name"] : std::string("");
        frameId = value.hasMember("frame_id") ? (std::string) value["frame_id"] : std::string("");
        port = value.hasMember("port") ? (std::string) value["port"] : std::string("");
        frequency = value.hasMember("frequency") ? (double) value["frequency"] : 0.0;
        minRange = value.hasMember("min_range") ? (double) value["min_range"] : 0.0;
        maxRange = value.hasMember("max_range") ? (double) value["max_range"] : 0.0;
        spreadAngle = value.hasMember("spread_angle") ? (double) value["spread_angle"] : 0.0;
    }
    ~DeviceConfig() = default;

    const std::string& getType() const {return type;}
    const std::string& getName() const {return name;}
    const std::string& getFrameId() const {return frameId;}
    const std::string& getPort() const {return port;}
    double getFrequency() const {return frequency;}
    double getMinRange() const { return minRange;}
    double getMaxRange() const { return  maxRange;}
    double getSpreadAngle() const { return spreadAngle;}
};

#endif //RXCPP_LANG_BRICKPI3UTILS_H
