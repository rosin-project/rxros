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

#include <iostream>
#include <algorithm>
#include <rxros.h>
#include <sensor_msgs/JointState.h>
using namespace rxcpp::operators;
using namespace rxros::operators;

auto tuple_2_joint_states = [](const auto& ljs, const auto& rjs) { // tuple: left joint state, right joint state
    static std::atomic<unsigned int> seqNo(0);
    sensor_msgs::JointState jointState;
    //jointState.header.frame_id = name;
    jointState.header.stamp = ros::Time::now();
    jointState.header.seq = seqNo++;
    jointState.name.push_back(ljs.name[0]);
    jointState.effort.push_back(ljs.effort[0]);
    jointState.position.push_back(ljs.position[0]); // rad
    jointState.velocity.push_back(ljs.velocity[0]); // rad/s
    jointState.name.push_back(rjs.name[0]);
    jointState.effort.push_back(rjs.effort[0]);
    jointState.position.push_back(rjs.position[0]); // rad
    jointState.velocity.push_back(rjs.velocity[0]); // rad/s
    return jointState;};

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "brickpi3_joint_states_publisher"); // Name of this node.

    const auto l_wheel_joint = rxros::parameter::get("/brickpi3/l_wheel_joint", "l_wheel_joint");
    const auto r_wheel_joint = rxros::parameter::get("/brickpi3/r_wheel_joint", "r_wheel_joint");

    rxros::logging().info() << "brickpi3_joint_states_publisher:";
    rxros::logging().info() << "l_wheel_joint: " << l_wheel_joint;
    rxros::logging().info() << "r_wheel_joint: " << r_wheel_joint ;

    const auto joint_state_observable = rxros::observable::from_topic<sensor_msgs::JointState>("/joint_state");
    const auto left_wheel_observable = joint_state_observable.filter([=](auto& jointState){return (jointState.name[0] == l_wheel_joint);});
    const auto right_wheel_observable = joint_state_observable.filter([=](auto& jointState){return (jointState.name[0] == r_wheel_joint);});
    left_wheel_observable.zip(tuple_2_joint_states, right_wheel_observable)
    | publish_to_topic<sensor_msgs::JointState>("/joint_states");

    rxros::spin();
}
