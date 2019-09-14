# RxROS

## Introduction

RxROS is new API for ROS based on the paradigm of reactive programming.
Reactive programming is an alternative to callback-based programming for implementing
concurrent message passing systems that emphasizes explicit data flow over control flow.
It makes the message flow and transformations of messages easy to capture in one place.
It eliminates problems with deadlocks and understanding how callbacks interact.
It helps you to make your nodes functional, concise, and testable.
RxROS aspires to the slogan ‘concurrency made easy’.


## Contents

   * [RxROS](#rxros)
      * [Introduction](#introduction)
      * [Setup and installation](#setup-and-installation)
         * [RxROS](#rxros-1)
         * [Acknowledgements](#acknowledgements)
      * [Example packages](#example-packages)
      * [Initial setup](#initial-setup)
         * [Syntax](#syntax)
         * [Example](#example)
      * [Spinning](#spinning)
         * [Syntax](#syntax-1)
         * [Example](#example-1)
      * [Parameters](#parameters)
         * [Syntax](#syntax-2)
         * [Example](#example-2)
      * [Logging](#logging)
         * [Syntax](#syntax-3)
         * [Example](#example-3)
      * [Observables](#observables)
         * [Observable from a Topic](#observable-from-a-topic)
            * [Syntax](#syntax-4)
            * [Example](#example-4)
         * [Observable from a transform listener](#observable-from-a-transform-listener)
            * [Syntax](#syntax-5)
         * [Observable from a Linux device](#observable-from-a-linux-device)
            * [Syntax](#syntax-6)
            * [Example](#example-5)
         * [Observable from a Yaml file](#observable-from-a-yaml-file)
            * [Syntax](#syntax-7)
            * [Example](#example-6)
      * [Operators](#operators)
         * [Publish to Topic](#publish-to-topic)
            * [Syntax:](#syntax-8)
            * [Example:](#example-7)
         * [Send Transform](#send-transform)
            * [Syntax:](#syntax-9)
         * [Call Service](#call-service)
            * [Syntax:](#syntax-10)
         * [Sample with Frequency](#sample-with-frequency)
            * [Syntax:](#syntax-11)
            * [Example:](#example-8)
      * [Example 1: A Keyboard Publisher](#example-1-a-keyboard-publisher)
      * [Example 2: A Velocity Publisher](#example-2-a-velocity-publisher)


## Setup and installation

In order to make use of this software you must install the following software on your computer:

 * Ubuntu Bionic (18.04)
 * ROS Melodic Morenia

Installation instructions for ROS Melodic on Ubuntu Bionic may be found [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

### RxROS
Finally, we have come to the RxROS project.
To install RxROS do the following:

```bash
# make sure we have the base setup file sourced
source /opt/ros/melodic/setup.bash

# choose any name you like for your workspace.
mkdir ~/rxros_ws
cd ~/rxros_ws

# populate the workspace with rxros packages
git clone https://github.com/rosin-project/rxros.git src/rxros

# install all required dependencies
rosdep update
rosdep install --from-paths ~/rxros_ws/src --ignore-src

# build the workspace
catkin_make
```

To start using RxROS you must execute the following commands:

```bash
source ~/rxros_ws/devel/setup.bash
mkdir -p ~/my_ws/src
cd ~/my_ws/src
catkin_create_pkg my_pkg std_msgs rxros
# create the files you need in your new package ...
cd ~/ws
catkin_make
```

### Acknowledgements

The RxROS library depends on and uses the following software:<br>

1. Ubuntu Bionic 18.04<br>
2. ROS Melodic v12<br>
3. Reactive C++ v2<br>
https://github.com/ReactiveX/RxCpp<br>
Released under the Microsoft Open Source Code of Conduct.<br>
The RxCpp library (header files) is included in the RxROS distribution.<br>
4. roscpp meets c++14 now!! by Takashi Ogura (OTL)<br>
https://github.com/OTL/roscpp14<br>
Released under Apache License 2.0<br>
Ideas from the library has been used in RxROS.


## Example packages

Some example packages showcasing the use of RxROS may be found in the [rxros_examples](https://github.com/rosin-project/rxros_examples) repository.

Refer to the `README` in that repository for additional setup and installation instructions.


Now, lets look at the language in more details.
The RxROS provides simple access to ROS via a set of classes.
The classes provides more precisely an extension to RxCpp that
gives simple access to ROS.<br>

## Initial setup

The most fundamental RxROS program is the initialization of a ROS node.
This is done simply by calling the rxros::init function. The rxros::init function takes three arguments:
The number of command line arguments the node or program was called with (argc), the actual arguments (argv)
and the name of the node. argc and argv are inherited directly from the main function and all three arguments
are used to initialize a ROS node. Failure to initialize a node with rxros::init will cause all interaction
with ROS to fail.

### Syntax

```cpp
rxros::init(argc, argv, "Name_of_ROS_node");
```

### Example

```cpp
#include <rxros.h>
int main(int argc, char** argv) {'
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.
    // ...
}
```

## Spinning

The other fundamental function of RxROS is rxros::spin. The function blocks the main thread until it is shutdown or terminated.
This means that the rxros::spin function always should be placed at the very bottom of the main function.
The other purpose of the rxros::spin function is to dispatch messages from ROS topics to the appropriate RxROS observables.
Failure to omit the rxros::spin function will cause the program to terminate immediately and observables based on ROS topics
will not emit any events.

The Example below shows a minimal RxROS program that creates a ROS node named “my_node”.
The program will due to rxros:spin() continue to run until it is either shutdown or terminated.

### Syntax

```cpp
void rxros::spin();
```

### Example

```cpp
#include <rxros.h>
int main(int argc, char** argv) {
    rxros::init(argc, argv, "my_node"); // Name of this node.
    // ...  
    rxros::spin(); // block the main thread until it is terminated.
}
```

## Parameters

ROS provides through the parameters interface a simple way to customize a node without having to recompile
the source each time there is a change to its configuration. The RxROS parameter interface provides easy
access to the ROS parameter server through an overloaded set of get functions. Each function takes as argument
the name of the parameter to be looked up and a default value that is returned if the specified parameter
was not found. The returned result is of the same type as the default value. Failure to specify the correct
parameter type will either cause a wrong conversion or the program may simply crash.

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

Logging is fundamentally a debugging facility that allows the programmer to print out information
about the robot’s internal state. RxROS provides five logging levels where each level indicates the
severity of the problem. Each loglevel returns as shown a reference to a logging object which actually
is an std::ostringstream. This means that the well-known C++ stream insertion operator “<<” can be
used to compose the logging messages. RxROS logging will be fully backwards compatible with ROS.
This means that all the functionality provided by the ROS logging framework is also available to RxROS.

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
    //...
    rxros::spin();
}
```

## Observables

Observables are asynchronous message streams. They are the fundamental data structure used by RxROS.
As soon as we have the observables RxCpp will provide us with a number of functions and operators
to manipulate the streams.

### Observable from a Topic

An observable data stream is created from a topic simply by calling the rxros::observable::from_topic function.
The function takes two arguments a name of the topic and an optional queue size.
In order to use the rxros::observable::from_topic function it is important also to specify the type of the topic messages.

The example below demonstrates how two ROS topics named “/joystick” and “/keyboard” are turned into two observable streams
by means of the rxros::observable::from_topic function and then merged together into a new observable message stream named
teleop_obsrv. Observe the use of the map operator: Since teleop_msgs::Joystick and teleop_msgs::Keyboard are different
message types it is not possible to merge them directly. The map operator solves this problem by converting each
teleop_msgs::Joystick and teleop_msgs::Keyboard message into a simple integer that represents the low level event of
moving the joystick or pressing the keyboard.

The pipe operator “|” is a specialty of RxCpp that is used as a simple mechanism to compose operations on
observable message streams. The usual “.” notation could have been used just as vel, but it’s common to use
the pipe operator “|” in RxCpp.

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

### Observable from a transform listener

A transform listener listens as the name indicates to broadcasted transformations, or more specific it listens to
broadcasted transformations from a specified parent frame id to a specified child frame id.
The rxros::observable::from_transform turns these transformations into an observable message stream of
type tf::StampedTransform. The rxros::observable::from_transform takes three arguments: the parent frameId and child frameId
and then an optional frequency. The frequency specifies how often the rxros::observable::from_transform will perform
a lookup of the broadcasted transformations.

#### Syntax

```cpp
auto rxros::observable::from_transform(const std::string& parent_frameId, const std::string& child_frameId, const double frequency = 10.0)
```

### Observable from a Linux device

The function rxros::observable::from_device will turn a Linux block or character device like “/dev/input/js0” into an
observable message stream. rxros::observable::from_device has as such nothing to do with ROS, but it provides an
interface to low-level data types that are needed in order to create e.g. keyboard and joystick observables.
The rxros::observable::from_device takes as argument the name of the device and a type of the data that are read
from the device.

The example below shows what it takes to turn a stream of low-level joystick events into an observable message stream
and publish them on a ROS topic. First an observable message stream is created from the device “/dev/input/js0”.
Then is is converted it to a stream of ROS messages and finally the messages are published to a ROS topic.
Three simple steps, that’s it! <br>

#### Syntax

```cpp
auto rxros::observable::from_device<device_type>(const std::string& device_name)
```

#### Example

```cpp
int main(int argc, char** argv) {
    rxros::init(argc, argv, "joystick_publisher"); // Name of this node.
    //...
    rxros::observable::from_device<joystick_event>("/dev/input/js0")
        | map(joystickEvent2JoystickMsg)
        | publish_to_topic<teleop_msgs::Joystick>("/joystick");
    //...
    rxros::spin();
}
```

### Observable from a Yaml file

This section describes how to turn a Yaml configuration file into an observable message stream.
ROS uses the Yaml format to configure various packages. An example of a Yaml file is given below:
It’s a configuration of the sensors and actuators that are used on a BrickPi3 robot.

```yaml
brickpi3_robot:
  - type: motor
    name: r_wheel_joint
    port: PORT_D
    frequency: 20.0

  - type: color
    frame_id: color_link
    name: color_sensor
    port: PORT_4
    frequency: 2.0

  - type: touch
    frame_id: touch_link
    name: touch_sensor
    port: PORT_1
    frequency: 2.0
```

The Yaml file defines a configuration of the BrickPi3 motor, color and touch sensor.
The rxros::observable::from_yaml function will turn the Yaml configuration file into
an observable data stream with three elements: One element for each device.

The example below demonstrates how to use the rxros::observable::from_yaml function.
As soon as we subscribe to the observable it will start to emit events (on_next events)
in form of configurations of type XmlRpc::XmlRpcValue for each device. The device can
then be used to lookup information about its type, name, port and frequency.
The DeveiceConfig is a simple helper class that will take the configurations and provide
simple access functions such as device.getType() which is equal to config["Type"].

#### Syntax

```cpp
auto rxros::observable::from_yaml(const std::string& namespace)
```

#### Example

```cpp
#include <rxros.h>
int main(int argc, char** argv) {
    rxros::init(argc, argv, "brickpi3"); // Name of this node.
    //...
    rxros::observable::from_yaml("/brickpi3/brickpi3_robot").subscribe(
        [=](auto config) { // on_next event
            DeviceConfig device(config);
            if (device.getType() == "motor") {
                rxros::logging().debug() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();
    //..
    rxros::spin();
}
```

```cpp
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
```

## Operators

One of the primary advantages of stream oriented processing is the fact that we can apply functional programming
primitives on them. RxCpp operators are nothing but filters, transformations, aggregations and reductions of the
observable message streams we created in the previous section.

### Publish to Topic

rxros::operators::publish_to_topic is a rather special operator. It does not modify the message steam -
it is in other words an identity function/operator. It will however take each message from the
stream and publish it to a specific topic. This means that it is perfectly possible to continue
modifying the message stream after it has been published to a topic. This will allow us to e.g.
send transform broadcasts or even publish the messages to other topics.

#### Syntax:

```cpp
auto rxros::operators::publish_to_topic<topic_type>(const std::string &topic, const uint32_t queue_size = 10)
```

#### Example:

```cpp
int main(int argc, char** argv) {
    rxros::init(argc, argv, "joystick_publisher"); // Name of this node.
    //...
    rxros::observable::from_device<joystick_event>("/dev/input/js0")
        | map(joystickEvent2JoystickMsg)
        | publish_to_topic<teleop_msgs::Joystick>("/joystick");
    //...
    rxros::spin();
}
```

### Send Transform

The rxros::operators::send_transform operator works exactly the same ways as the rxros::operators::publish_to_topic operator.
It does not modify the message steam it is operating on, but it will take each message and broadcast it to any listeners.
The rxros::operators::send_transform comes in two variants: One that takes no arguments and operates on message streams of
type tf::StampedTranformation and one that takes the parent frame id and child frame id as argument and operates on message
streams of type tf::Transform. The later will convert the tf::Transform messages into tf::StampedTranformation messages so
that both operators broadcast the same message type.

#### Syntax:

```cpp
auto rxros::operators::send_transform();
auto rxros::operators::send_transform(const std::string& parent_frameId, const std::string& child_frameId);
```

### Call Service

Besides the publish/subscribe model, ROS also provides a request/reply model that allows a remote procedure call (RPC)
to be send from one node (request) and handled by another node (reply) - it is a typical client-server mechanism that
can be useful in distributed systems.

RxROS only provides a means to send a request, i.e. the client side. The server side will have to be created exactly
the same way as it is done it ROS. To send a request the  rxros::operators::call_service operator is called. It take
a service name as argument and service type that specifies the type of the observable message stream the operation
was applied on. The service type consists of a request and response part. The request part must be filled out prior
to the service call and the result will be a new observable stream where the response part has been filled out by the
server part.

#### Syntax:

```cpp
auto rxros::operators::call_service<service_type>(const std::string& service_name)
```

### Sample with Frequency

The operator rxros::operators::sample_with_frequency will at regular intervals emit the last element or message of the
observable message stream it was applied on - that is independent of whether it has changed or not. This means that
the observable message stream produced by rxros::operators::sample_with_frequency may contain duplicated messages if
the frequency is too high and it may miss messages in case the frequency is too low. This is the preferred way in ROS
to publish messages on a topic and therefore a needed operation.

The operation operator rxros::operators::sample_with_frequency comes in two variants.
One that is executing in the current thread and one that is executing in a specified thread also known as a
coordination in RxCpp.

#### Syntax:

```cpp
auto rxros::operators::sample_with_frequency(const double frequency)
auto rxros::operation::sample_with_frequency(const double frequency, Coordination coordination)
```

#### Example:

```cpp
int main(int argc, char** argv) {
    rxros::init(argc, argv, "joystick_publisher"); // Name of this node.
    //...
    | sample_with_frequency(frequencyInHz)
    | publish_to_topic<geometry_msgs::Twist>("/cmd_vel");
    //...
    rxros::spin();
}
```

## Example 1: A Keyboard Publisher

The following example is a full implementation of a keyboard publisher
that takes input from a Linux block device and publishes the low-level
keyboard events to a ROS topic '/keyboard'

```cpp
#include <rxros.h>
#include <teleop_msgs/Keyboard.h>
#include "KeyboardPublisher.h"
using namespace rxcpp::operators;
using namespace rxros::operators;

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "keyboard_publisher"); // Name of this Node.

    const auto keyboardDevice = rxros::parameter::get("/keyboard_publisher/device", "/dev/input/event1");
    rxros::logging().info() << "Keyboard device: " << keyboardDevice;

    auto keyboardEvent2KeyboardMsg = [](const auto keyboardEvent) {
        auto makeKeyboardMsg = [=] (auto event) {
            teleop_msgs::Keyboard keyboardMsg;
            keyboardMsg.time = ros::Time(keyboardEvent.time.tv_sec, keyboardEvent.time.tv_usec);
            keyboardMsg.event = event;
            return keyboardMsg;};
        if ((keyboardEvent.type == EV_KEY) && (keyboardEvent.value != REP_DELAY)) {
            if (keyboardEvent.code==KEY_UP)
                return makeKeyboardMsg(KB_EVENT_UP);
            else if (keyboardEvent.code==KEY_LEFT)
                return makeKeyboardMsg(KB_EVENT_LEFT);
            else if (keyboardEvent.code==KEY_RIGHT)
                return makeKeyboardMsg(KB_EVENT_RIGHT);
            else if (keyboardEvent.code==KEY_DOWN)
                return makeKeyboardMsg(KB_EVENT_DOWN);
            else if (keyboardEvent.code==KEY_SPACE)
                return makeKeyboardMsg(KB_EVENT_SPACE);
        }
        return makeKeyboardMsg(KB_EVENT_NONE);};

    rxros::observable::from_device<input_event>(keyboardDevice)
        | map(keyboardEvent2KeyboardMsg)
        | publish_to_topic<teleop_msgs::Keyboard>("/keyboard");

    rxros::logging().info() << "Spinning keyboard_publisher...";
    rxros::spin();
}

```

## Example 2: A Velocity Publisher

The following example is a full implementation of a velocity publisher
that takes input from a keyboard and joystick and publishes Twist messages
on the /cmd_vel topic:<br>

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
