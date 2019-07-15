# RxROS

RxROS is new API for ROS based on the paradigm of reactive programming.
Reactive programming is an alternative to callback-based programming for implementing
concurrent message passing systems that emphasizes explicit data flow over control flow.
It makes the message flow and transformations of messages easy to capture in one place. 
It eliminates problems with deadlocks and understanding how callbacks interact.
It helps you to make your nodes functional, concise, and testable. 
RxROS aspires to the slogan ‘concurrency made easy.’ 

## Setup and installation

In order to make use of this software you must
install the following software on your computer:

### Ubuntu Bionic (18.04)

You can download an image of the Ubuntu Bionic Linux distribution at<br>
https://www.ubuntu.com/#download<br>
You may in addition find the following packages usefull

```bash
sudo apt-get install git doxygen graphviz-* meld cmake
sudo apt-get install emacs qtcreator qt5-*
sudo apt-get install tree gimp
sudo apt-get install liburdfdom-tools
sudo apt-get install libcanberra-gtk-module
```

### ROS Melodic Morenia

Installation instruction of how to install ROS Melodic Morenia can be found at<br>
http://wiki.ros.org/melodic/Installation/Ubuntu<br>
Execute the following commands to install ROS Melodic Morenia:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-keyboard
sudo apt-get install ros-melodic-navigation
```

### Reactive C++

Reactive C++ is also a github project. It can be found at<br>
https://github.com/ReactiveX/RxCpp<br>
To install RxCpp execute the following commands:

```bash
git clone --recursive  https://github.com/ReactiveX/RxCpp.git 
cd RxCpp 
mkdir -p projects/build 
cd projects/build 
cmake -G"Unix Makefiles" -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=RelWithDebInfo -B. ../CMake 
make #optional, will take long time to execute.
sudo make install 
cd ..
```

After the installation has completed you can remove the RxCpp directory.
The needed C++ header files have been installed in /usr/include/rxcpp.

### RxROS

Finally, we have come to the RxROS project. To install RxROS do the following:

```bash
git clone https://github.com/rosin-project/rx_ros.git
cd rx_ros
sudo install.sh
cd .. 
```

The RxROS language depends on the following software:<br>
1. Ubuntu Binoic 18.04
2. ROS Melodic v12
3. Reactive C++ v2

The software must be installed as described above.<br>
The RxROS itself consist of a singe file named rxros.h.
The file must be installed in /usr/local/include.<br>

Now, lets look at the language in more details.
The RxROS provides simple access to ROS via a set of classes.
The classes provides more precisely an extension to RxCpp that
gives simple access to ROS.<br>

## Initial setup

A RxROS program is in principle a ROS node,
so the first step is not surprisingly to initialise it and specify the node name.
This is done by means of the init function<br>

### Syntax

```cpp
rxros::init(argc, argv, "Name_of_ROS_node");
```

### Example

```cpp
#include <rxros.h>
int main(int argc, char** argv) {'
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.

    // ... here 

    rxros::spin();
}
```

## Parameters

### Syntax

```cpp
auto rxros::parameter::get<param_type>(const std::string& name, const param_type& defaultValue)
auto rxros::parameter::get(const std::string& name, const int defaultValue)
auto rxros::parameter::get(const std::string& name, const double defaultValue)
auto rxros::parameter::get(const std::string& name, const char* defaultValue)
auto rxros::parameter::get(const std::string& name, const std::string& defaultValue)
```

### Example

```cpp
#include <rxros.h>
int main(int argc, char** argv) {
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.
    //...
    const auto frequency = rxros::parameter::get("/velocity_publisher/frequency", 10.0); // hz
    const auto minVelLinear = rxros::parameter::get("/velocity_publisher/min_vel_linear", 0.04); // m/s
    const auto maxVelLinear = rxros::parameter::get("/velocity_publisher/max_vel_linear", 0.10); // m/s
    //...
    rxros::spin();
}
```

## Logging

### Syntax

```cpp
rxros::logging& rxros::logging().debug();
rxros::logging& rxros::logging().info();
rxros::logging& rxros::logging().warn();
rxros::logging& rxros::logging().error();
rxros::logging& rxros::logging().fatal();
```

### Example

```cpp
#include <rxros.h>
int main(int argc, char** argv) {
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.
    //...
    rxros::logging().debug() << "frequency: " << frequency;
    rxros::logging().info() << "min_vel_linear: " << minVelLinear << " m/s";
    rxros::logging().warn() << "max_vel_linear: " << maxVelLinear << " m/s";
    rxros::logging().error() << "min_vel_angular: " << minVelAngular << " rad/s";
    rxros::logging().fatal() << "max_vel_angular: " << maxVelAngular << " rad/s";
```

## Observables

### Topics

#### Syntax

```cpp
auto rxros::observable::from_topic<topic_type>(const std::string& topic, const uint32_t queue_size = 10)
```

#### Example

```cpp
int main(int argc, char** argv) {
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.
    //...
    auto joy_obsrv = rxros::observable::from_topic<teleop_msgs::Joystick>("/joystick")
        | map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto key_obsrv = rxros::observable::from_topic<teleop_msgs::Keyboard>("/keyboard")
        | map([](teleop_msgs::Keyboard key) { return key.event; });
    auto teleop_obsrv = joyObsrv.merge(keyObsrv);
    //...
    rxros::spin();
}
```

### Broadcasters

#### Syntax

```cpp
auto rxros::observable::from_transform(const std::string& parent_frameId, const std::string& child_frameId, const double frequency = 10.0)
```

#### Example

```cpp
```


### Services

#### Syntax

```cpp
```

#### Example

```cpp
```


## Operators


## Example
The following example is a full implementation of a velocity publisher
that takes input from a keyboard and joystick and publishes Twist messages
on the /cmd_vel topic:

```cpp
#include <rxros.h>
#include <teleop_msgs/Joystick.h>
#include <teleop_msgs/Keyboard.h>
#include <geometry_msgs/Twist.h>
#include "JoystickPublisher.h"
#include "KeyboardPublisher.h"


int main(int argc, char** argv) {
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.

    const auto frequencyInHz = rxros::Parameter::get("/velocity_publisher/frequency", 10.0); // hz
    const auto minVelLinear = rxros::Parameter::get("/velocity_publisher/min_vel_linear", 0.04); // m/s
    const auto maxVelLinear = rxros::Parameter::get("/velocity_publisher/max_vel_linear", 0.10); // m/s
    const auto minVelAngular = rxros::Parameter::get("/velocity_publisher/min_vel_angular", 0.64); // rad/s
    const auto maxVelAngular = rxros::Parameter::get("/velocity_publisher/max_vel_angular", 1.60); // rad/s
    const auto deltaVelLinear = (maxVelLinear - minVelLinear) / 10.0;
    const auto deltaVelAngular = (maxVelAngular - minVelAngular) / 10.0;

    rxros::Logging().info() << "frequency: " << frequencyInHz;
    rxros::Logging().info() << "min_vel_linear: " << minVelLinear << " m/s";
    rxros::Logging().info() << "max_vel_linear: " << maxVelLinear << " m/s";
    rxros::Logging().info() << "min_vel_angular: " << minVelAngular << " rad/s";
    rxros::Logging().info() << "max_vel_angular: " << maxVelAngular << " rad/s";

    auto adaptVelocity = [=] (auto newVel, auto minVel, auto maxVel, auto isIncrVel) {
        if (newVel > maxVel)
            return maxVel;
        else if (newVel < -maxVel)
            return -maxVel;
        else if (newVel > -minVel && newVel < minVel)
            return (isIncrVel) ? minVel : -minVel;
        else
            return newVel;};

    auto teleop2VelTuple = [=](const auto& prevVelTuple, const int event) {
        const auto prevVelLinear = std::get<0>(prevVelTuple);  // use previous linear and angular velocity
        const auto prevVelAngular = std::get<1>(prevVelTuple); // to calculate the new linear and angular velocity.
        if (event == JS_EVENT_BUTTON0_DOWN || event == JS_EVENT_BUTTON1_DOWN || event == KB_EVENT_SPACE)
            return std::make_tuple(0.0, 0.0); // Stop the robot
        else if (event == JS_EVENT_AXIS_UP || event == KB_EVENT_UP)
            return std::make_tuple(adaptVelocity((prevVelLinear + deltaVelLinear), minVelLinear, maxVelLinear, true), prevVelAngular); // move forward
        else if (event == JS_EVENT_AXIS_DOWN || event == KB_EVENT_DOWN)
            return std::make_tuple(adaptVelocity((prevVelLinear - deltaVelLinear), minVelLinear, maxVelLinear, false), prevVelAngular); // move backward
        else if (event == JS_EVENT_AXIS_LEFT || event == KB_EVENT_LEFT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular + deltaVelAngular), minVelAngular, maxVelAngular, true)); // move left
        else if (event == JS_EVENT_AXIS_RIGHT || event == KB_EVENT_RIGHT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular - deltaVelAngular), minVelAngular, maxVelAngular, false));}; // move right

    auto velTuple2TwistMsg = [](auto velTuple) {
        geometry_msgs::Twist vel;
        vel.linear.x = std::get<0>(velTuple);
        vel.angular.z = std::get<1>(velTuple);
        return vel;};

    auto joyObsrv = rxros::Observable::fromTopic<teleop_msgs::Joystick>("/joystick") // create an Observable stream from "/joystick" topic
        | map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto keyObsrv = rxros::Observable::fromTopic<teleop_msgs::Keyboard>("/keyboard") // create an Observable stream from "/keyboard" topic
        | map([](teleop_msgs::Keyboard key) { return key.event; });
    joyObsrv.merge(keyObsrv)                                  // merge the joystick and keyboard messages into an Observable teleop stream.
        | scan(std::make_tuple(0.0, 0.0), teleop2VelTuple)    // turn the teleop stream into a linear and angular velocity stream.
        | map(velTuple2TwistMsg)                              // turn the linear and angular velocity stream into a Twist stream.
        | sample_with_frequency(frequencyInHz)                // take latest Twist msg and populate it with the specified frequency.
        | publish_to_topic<geometry_msgs::Twist>("/cmd_vel"); // publish the Twist messages to the topic "/cmd_vel"

    rxros::Logging().info() << "Spinning velocity_publisher ...";
    rxros::spin();
}
```
<br>
