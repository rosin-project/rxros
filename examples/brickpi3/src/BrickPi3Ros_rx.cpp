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
#include <sensor_msgs/JointState.h>
#include <brickpi3_msgs/Contact.h>
#include <brickpi3_msgs/Color.h>
#include <brickpi3_msgs/Range.h>
#include <brickpi3_msgs/JointCommand.h>
#include "BrickPi3Utils.h"
#include "BrickPi3Observable_rx.h"
using namespace rxcpp::operators;
using namespace rxros::operators;
using namespace brickpi3::operators;


template <class Observable>
auto join_with_latest_from(const Observable& observable) {
    return [=](auto&& source) {
        return source.with_latest_from (
            [=](const auto o1, const auto o2) {
                return std::make_tuple(o1, o2);}, observable);};}


int main(int argc, char** argv) {
    rxros::init(argc, argv, "brickpi3"); // Name of this node.

    auto touchEvent2ContactMsg = [] (const std::string& frameId) {
        return [=](const sensor_touch_t& touchEvent) {
            static int seqNo = 0;
            brickpi3_msgs::Contact contact;
            contact.header.frame_id = frameId;
            contact.header.stamp = ros::Time::now();
            contact.header.seq = seqNo++;
            contact.contact = touchEvent.pressed;
            return contact;};};

    auto colorEvent2ColorMsg = [] (const std::string& frameId) {
        return [=](const sensor_color_t& colorEvent) {
            static int seqNo = 0;
            brickpi3_msgs::Color color;
            color.header.frame_id = frameId;
            color.header.stamp = ros::Time::now();
            color.header.seq = seqNo++;
            color.r = static_cast<double>(colorEvent.reflected_red);
            color.g = static_cast<double>(colorEvent.reflected_green);
            color.b = static_cast<double>(colorEvent.reflected_blue);
            color.intensity = static_cast<double>(colorEvent.ambient);
            return color;};};

    auto ultrasonicEvent2RangeMsg = [] (const std::string& frameId, const double minRange, const double maxRange, const double spreadAngle) {
        return [=](const sensor_ultrasonic_t& ultrasonicEvent) {
            static int seqNo = 0;
            brickpi3_msgs::Range range;
            range.header.frame_id = frameId;
            range.header.stamp = ros::Time::now();
            range.header.seq = seqNo++;
            range.range = ultrasonicEvent.cm / 100.0;
            range.range_min = minRange;
            range.range_max = maxRange;
            range.spread_angle = spreadAngle;
            return range;};};

    auto motorEvent2PosVelTimeTuple = [](const auto& prevPosVelTimeTuple, const actuator_motor_t& motorEvent) {
        const auto prevTime = std::get<0>(prevPosVelTimeTuple);
        const auto prevPos = std::get<1>(prevPosVelTimeTuple);
        const auto prevVel = std::get<2>(prevPosVelTimeTuple);
        const auto currTime = ros::Time::now();
        const auto currPos = static_cast<double>(motorEvent.motorPosition) * M_PI / 180.0 / 3.0; // 3.0 is gearing ratio
        const auto currVel = (currPos - prevPos)/(currTime - prevTime).toSec();
        return std::make_tuple(currTime, currPos, currVel);};

    auto posVelTimeTuple2JointStateMsg = [](const std::string& name) {
        return [=](const auto& tuple) {
            static int seqNo = 0;
            const auto posVelTimeTuple = std::get<0>(tuple);
            const auto position = std::get<1>(posVelTimeTuple);
            const auto velocity = std::get<2>(posVelTimeTuple);
            const auto effort = std::get<1>(tuple).effort;
            sensor_msgs::JointState jointState;
            //jointState.header.frame_id = name;
            jointState.header.stamp = ros::Time::now();
            jointState.header.seq = seqNo++;
            jointState.name.push_back(name);
            jointState.effort.push_back(effort);
            jointState.position.push_back(position); // rad
            jointState.velocity.push_back(velocity); // rad/s
            return jointState;};};

    auto jointCmdObserv = rxros::observable::from_topic<brickpi3_msgs::JointCommand>("/joint_command");

    rxros::observable::from_yaml("/brickpi3/brickpi3_robot").subscribe(
        [=](auto config) { // on_next event
            DeviceConfig device(config);
            if (device.getType() == "motor") {
                rxros::logging().debug() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();

                brickpi3::observable::motor(device.getName(), device.getPort(), device.getFrequency())
                | scan(std::make_tuple(ros::Time::now(), 0.0, 0.0), motorEvent2PosVelTimeTuple)
                | join_with_latest_from(
                    jointCmdObserv
                    | filter([=](const auto& jointCmd){ return (jointCmd.name == device.getName()); })
                    | set_motor_power(device.getPort()))
                | map(posVelTimeTuple2JointStateMsg(device.getName()))
                | publish_to_topic<sensor_msgs::JointState>("/joint_state");
            }
            else if (device.getType() == "ultrasonic") {
                rxros::logging().debug() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();

                brickpi3::observable::ultrasonic_sensor(device.getName(), device.getPort(), device.getFrequency())
                | map(ultrasonicEvent2RangeMsg(device.getFrameId(), device.getMinRange(), device.getMaxRange(), device.getSpreadAngle()))
                | publish_to_topic<brickpi3_msgs::Range>("/" + device.getName());
            }
            else if (device.getType() == "color") {
                rxros::logging().debug() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();

                brickpi3::observable::color_sensor(device.getName(), device.getPort(), device.getFrequency())
                | map(colorEvent2ColorMsg(device.getFrameId()))
                | publish_to_topic<brickpi3_msgs::Color>("/" + device.getName());
            }
            else if (device.getType() == "touch") {
                rxros::logging().debug() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();

                brickpi3::observable::touch_sensor(device.getName(), device.getPort(), device.getFrequency())
                | map(touchEvent2ContactMsg(device.getFrameId()))
                | publish_to_topic<brickpi3_msgs::Contact>("/" + device.getName());
            }},
        [](){}); // on completed event

    rxros::spin();
}
