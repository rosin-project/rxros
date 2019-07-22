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

#include <mutex>
#include <stdio.h>
#include <string>
#include <Scheduler.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <brickpi3_msgs/JointCommand.h>
#include "Node.h"


class BrickPi3BaseController {
private:
    std::mutex mutex;
    ros::Publisher jointCommandPublisher;
    ros::Subscriber cmdVelSubscriber;
    ros::Subscriber jointStatesSubscriber;
    unsigned int seqNo;
    std::string lWheelJoint;
    std::string rWheelJoint;
    double wheelRadius;
    double wheelBasis;
    double lastDesiLinVel;
    double lastDesiAngVel;
    double desiLWheelVel;
    double desiRWheelVel;
    double factorLWheel;
    double factorRWheel;

    void cmdVelSubscriberCB(const geometry_msgs::Twist& jointStates);
    void jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates);

public:
    BrickPi3BaseController(int argc, char** argv);
    virtual ~BrickPi3BaseController() {};
    void run() {ros::spin();}
};

BrickPi3BaseController::BrickPi3BaseController(int argc, char** argv) :
    jointCommandPublisher(Node::getHandle().advertise<brickpi3_msgs::JointCommand>("/joint_command", 10)),
    cmdVelSubscriber(Node::getHandle().subscribe("/cmd_vel", 10, &BrickPi3BaseController::cmdVelSubscriberCB, this)),
    jointStatesSubscriber(Node::getHandle().subscribe("/joint_states", 10, &BrickPi3BaseController::jointStatesSubscriberCB, this)),
    seqNo(0),
    lastDesiLinVel(0.0),
    lastDesiAngVel(0.0),
    desiLWheelVel(0.0),
    desiRWheelVel(0.0),
    factorLWheel(0.0),
    factorRWheel(0.0)
{
    Node::getHandle().param<std::string>("/brickpi3_base_controller/l_wheel_joint", lWheelJoint, "l_wheel_joint");
    Node::getHandle().param<std::string>("/brickpi3_base_controller/r_wheel_joint", rWheelJoint, "r_wheel_joint");
    Node::getHandle().param("/brickpi3_base_controller/wheel_radius", wheelRadius, 0.028); // m
    Node::getHandle().param("/brickpi3_base_controller/wheel_basis", wheelBasis, 0.0625); // m

    printf("l_wheel_joint: %s\n", lWheelJoint.c_str());
    printf("r_wheel_joint: %s m/s\n", rWheelJoint.c_str());
    printf("wheel_radius: %f rad/s\n", wheelRadius);
    printf("wheel_basis: %f rad/s\n", wheelBasis);
}

void BrickPi3BaseController::cmdVelSubscriberCB(const geometry_msgs::Twist& twist)
{
    std::lock_guard<std::mutex> guard(mutex);

    double desiLinVel = twist.linear.x;
    double desiAngVel = twist.angular.z;
    if ((desiLinVel != lastDesiLinVel) || (desiAngVel != lastDesiAngVel)) {
        desiLWheelVel = desiLinVel - desiAngVel * wheelBasis; // m/s
        desiRWheelVel = desiLinVel + desiAngVel * wheelBasis; // m/s
//        (self.factor_l_joint, self.factor_r_joint) = get_default_factor(self.vel_l_joint_desi, self.vel_r_joint_desi)
        lastDesiLinVel = desiLinVel;
        lastDesiAngVel = desiAngVel;
    }
}

void BrickPi3BaseController::jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates)
{
    std::lock_guard<std::mutex> guard(mutex);

    double currLWheelVel = 0.0; // rad/s
    double currRWheelVel = 0.0; // rad/s
    for (int i = 0; i < jointStates.name.size(); i++) {
        if (lWheelJoint.compare(jointStates.name[i]) == 0) {
            currLWheelVel = jointStates.velocity[i];
        }
        else if (rWheelJoint.compare(jointStates.name[i]) == 0) {
            currRWheelVel = jointStates.velocity[i];
        }
    }
    currLWheelVel *= wheelRadius; // m/s
    currRWheelVel *= wheelRadius; // m/s

    if (currLWheelVel != 0.0) { // left wheel is turning
        double factor = desiLWheelVel / currLWheelVel; // perfect is 1.0
        factorLWheel += (factor > 1.0) ? 0.01 : -0.01;
    }
    else if (desiLWheelVel != 0.0) { // wheel is not turning should it?
        factorLWheel += 0.05;
    }

    if (currRWheelVel != 0.0) { // right wheel is turning
        double factor = desiRWheelVel / currRWheelVel; // perfect is 1.0
        factorRWheel += (factor > 1.0) ? 0.01 : -0.01;
    }
    else if (desiRWheelVel != 0.0) { // wheel is not turning should it?
        factorRWheel += 0.05;
    }

    // Wheel commands
    brickpi3_msgs::JointCommand lJointCommand;
    lJointCommand.name = lWheelJoint;
    lJointCommand.effort = desiLWheelVel * factorLWheel;
    jointCommandPublisher.publish(lJointCommand);

    brickpi3_msgs::JointCommand rJointCommand;
    rJointCommand.name = rWheelJoint;
    rJointCommand.effort = desiRWheelVel * factorRWheel;
    jointCommandPublisher.publish(rJointCommand);

//     rospy.loginfo("current %f m/s, %f m/s", vel_l_joint_curr, vel_r_joint_curr)
//     rospy.loginfo("desired %f m/s, %f m/s", self.vel_l_joint_desi, self.vel_r_joint_desi)
//     rospy.loginfo("factor %f, %f", self.factor_l_joint, self.factor_r_joint)
//     rospy.loginfo("cmd %f, %f", l_cmd.effort, r_cmd.effort)
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brickpi3_base_controller"); // Name of this node.
    BrickPi3BaseController BrickPi3BaseController(argc, argv);
    BrickPi3BaseController.run();
}
