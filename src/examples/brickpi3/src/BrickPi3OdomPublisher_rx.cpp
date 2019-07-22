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
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
using namespace rxcpp::operators;
using namespace rxros::operators;

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "brickpi3_odom_publisher"); // Name of this node.

    const auto l_wheel_joint = rxros::parameter::get("/brickpi3/l_wheel_joint", "l_wheel_joint");
    const auto r_wheel_joint = rxros::parameter::get("/brickpi3/r_wheel_joint", "r_wheel_joint");
    const auto wheel_radius = rxros::parameter::get("/brickpi3/wheel_radius", 0.028); // m
    const auto wheel_basis = rxros::parameter::get("/brickpi3/wheel_basis", 0.0625); // m

    rxros::logging().info() << "brickpi3_odom_publisher:";
    rxros::logging().info() << "l_wheel_joint: " << l_wheel_joint;
    rxros::logging().info() << "r_wheel_joint: " << r_wheel_joint ;
    rxros::logging().info() << "wheel_radius: " << wheel_radius ;
    rxros::logging().info() << "wheel_basis: " << wheel_basis ;

    auto stsTuple = [=](const auto& prevTuple, const auto& jointStates) {
        const auto x = std::get<0>(prevTuple);
        const auto y = std::get<1>(prevTuple);
        const auto th = std::get<2>(prevTuple);
        const auto last_lWheel_pos = std::get<5>(prevTuple);
        const auto last_rWheel_pos = std::get<6>(prevTuple);
        const auto last_time = std::get<7>(prevTuple);
        const auto curr_time = ros::Time::now();
        const auto curr_lWheel_pos = jointStates.position[0]; // first element is expected to be a left wheel
        const auto curr_rWheel_pos = jointStates.position[1]; // second element is expected to be a right wheel
        const auto delta_rWheel_pos = curr_rWheel_pos - last_rWheel_pos;
        const auto delta_lWheel_pos = curr_lWheel_pos - last_lWheel_pos;
        const auto velocity_x = (delta_rWheel_pos + delta_lWheel_pos) * wheel_radius / 2.0;
        const auto velocity_y = 0.0;
        const auto velocity_th = (delta_rWheel_pos - delta_lWheel_pos) * wheel_radius / (2.0 * wheel_basis);
        const auto dt = (curr_time - last_time).toSec();
        const auto dx = (velocity_x * cos(th) - velocity_y * sin(th)) * dt;
        const auto dy = (velocity_x * sin(th) + velocity_y * cos(th)) * dt;
        const auto dth = velocity_th * dt;
        return std::make_tuple(x + dx, y + dy, th + dy, velocity_x, velocity_th, curr_lWheel_pos, curr_rWheel_pos, curr_time);};

    auto stsTuple2Odometry = [](const auto& stsTuple) {  // stsTuple: x, y, theta θ, velocity x, velocity theta, left wheel pos, right wheel pos, current time
        static std::atomic<unsigned int> seqNo(0);
        const auto x = std::get<0>(stsTuple);
        const auto y = std::get<1>(stsTuple);
        const auto th = std::get<2>(stsTuple);
        const auto velocity_x = std::get<3>(stsTuple);
        const auto velocity_th = std::get<4>(stsTuple);
        const auto curr_time = std::get<7>(stsTuple);
        nav_msgs::Odometry odom;
        odom.header.stamp = curr_time;
        odom.header.seq = seqNo++;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x; //set the position
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
        odom.pose.covariance = {0.00001, 0.0,     0.0,     0.0,     0.0,     0.0,
                                0.0,     0.00001, 0.0,     0.0,     0.0,     0.0,
                                0.0,     0.0,     10.0000, 0.0,     0.0,     0.0,
                                0.0,     0.0,     0.0,     1.00000, 0.0,     0.0,
                                0.0,     0.0,     0.0,     0.0,     1.00000, 0.0,
                                0.0,     0.0,     0.0,     0.0,     0.0,     (velocity_th == 0.0) ? 0.00000000001 : 1.00000};
        odom.twist.twist.linear.x = velocity_x; //set the velocity
        odom.twist.twist.angular.z = velocity_th;
        return odom;};

    rxros::observable::from_topic<sensor_msgs::JointState>("/joint_states")
        | scan(std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ros::Time::now()), stsTuple) // tuple: x, y, theta θ, velocity x, velocity theta, left wheel pos, right wheel pos, current time
        | map(stsTuple2Odometry)
        | publish_to_topic<nav_msgs::Odometry>("/odom");

    rxros::spin();
}
