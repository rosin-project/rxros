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

#include <algorithm>
#include <rxros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <brickpi3_msgs/JointCommand.h>
using namespace rxcpp::operators;
using namespace rxros::operators;

enum WheelId{ LEFT_WHEEL = 0, RIGHT_WHEEL = 1};


int main(int argc, char** argv)
{
    rxros::init(argc, argv, "brickpi3_base_controller"); // Name of this node.

    const auto l_wheel_joint = rxros::parameter::get("/brickpi3/l_wheel_joint", "l_wheel_joint");
    const auto r_wheel_joint = rxros::parameter::get("/brickpi3/r_wheel_joint", "r_wheel_joint");
    const auto wheel_radius = rxros::parameter::get("/brickpi3/wheel_radius", 0.028); // m
    const auto wheel_basis = rxros::parameter::get("/brickpi3/wheel_basis", 0.0625); // m

    rxros::logging().info() << "brickpi3_base_controller:";
    rxros::logging().info() << "l_wheel_joint: " << l_wheel_joint;
    rxros::logging().info() << "r_wheel_joint: " << r_wheel_joint;
    rxros::logging().info() << "wheel_radius: " << wheel_radius;
    rxros::logging().info() << "wheel_basis: " << wheel_basis;

    auto adjust_factor = [] (const auto& curr_factor, const auto& curr_vel, const auto& desi_vel) {
        if (curr_vel != 0.0) // wheel is turning
            return curr_factor + ((desi_vel / curr_vel) > 1.0) ? 0.01 : -0.01;
        else if (desi_vel != 0.0) // wheel is not turning should it?
            return curr_factor + 0.05;
        else
            return curr_factor;};

    auto desired_wheel_vel = [=](const auto& wheel, const auto& cmd_vel) {
        if (wheel == LEFT_WHEEL)
            return cmd_vel.linear.x - cmd_vel.angular.z * wheel_basis; // m/s
        else if (wheel == RIGHT_WHEEL)
            return cmd_vel.linear.x + cmd_vel.angular.z * wheel_basis;}; // m/s

    auto curr_wheel_vel = [=](const auto& wheel, const auto& joint_states) {
        return joint_states.velocity[wheel] * wheel_radius;};

    auto update_effort = [=](const auto& prevTuple, const auto& tuple) {  // tuple: joint_states, cmd_vel
        const auto last_factor_lWheel = std::get<2>(prevTuple);
        const auto last_factor_rWheel = std::get<3>(prevTuple);
        const auto joint_states = std::get<0>(tuple);
        const auto cmd_vel = std::get<1>(tuple);
        const auto curr_factor_lWheel = adjust_factor(last_factor_lWheel, curr_wheel_vel(LEFT_WHEEL, joint_states), desired_wheel_vel(LEFT_WHEEL, cmd_vel));
        const auto curr_factor_rWheel = adjust_factor(last_factor_rWheel, curr_wheel_vel(RIGHT_WHEEL, joint_states), desired_wheel_vel(RIGHT_WHEEL, cmd_vel));
        return std::make_tuple(
            desired_wheel_vel(LEFT_WHEEL, cmd_vel) * curr_factor_lWheel, // effort for left wheel
            desired_wheel_vel(RIGHT_WHEEL, cmd_vel) * curr_factor_rWheel, // effort for right wheel
            curr_factor_lWheel,
            curr_factor_rWheel);};

    auto effort_2_joint_cmd = [=] (const auto& wheel) {
        return [=](const auto& tuple) {
            brickpi3_msgs::JointCommand jointCommand;
            jointCommand.name = (wheel == LEFT_WHEEL) ? l_wheel_joint : r_wheel_joint;
            jointCommand.effort = (wheel == LEFT_WHEEL) ? std::get<0>(tuple) : std::get<1>(tuple);
            return jointCommand;};};

    auto joint_states_observable = rxros::observable::from_topic<sensor_msgs::JointState>("/joint_states");
    auto cmd_vel_observable = rxros::observable::from_topic<geometry_msgs::Twist>("/cmd_vel");
    auto effort_observable = joint_states_observable.with_latest_from([](const auto& js, const auto& cv) {return std::make_tuple(js, cv);}, cmd_vel_observable)
        | scan(std::make_tuple(0.0, 0.0, 0.0, 0.0), update_effort); // tuple: effort left wheel, effort right wheel, factor left wheel, factor right wheel.

    effort_observable
    | map(effort_2_joint_cmd(LEFT_WHEEL))
    | publish_to_topic<brickpi3_msgs::JointCommand>("/joint_command");

    effort_observable
    | map(effort_2_joint_cmd(RIGHT_WHEEL))
    | publish_to_topic<brickpi3_msgs::JointCommand>("/joint_command");

    rxros::spin();
}
