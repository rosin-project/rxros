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

#ifndef BRICKPI3_BRICKPI3OBSERVABLE_RX_H
#define BRICKPI3_BRICKPI3OBSERVABLE_RX_H

#include <string>
#include <rxros.h>
#include "BrickPi3Device.h"

struct actuator_motor_t
{
    uint8_t motorState;// Variable for reading motor motorState
    int8_t motorPower;// Variable for reading motor powers
    int32_t motorPosition;  // Variable for reading motor encoder positions
    int16_t motorDPS; // Variable for reading motor speeds (Degrees Per Second)
};


namespace brickpi3
{
    class observable
    {
    private:
        observable() = default;

    public:
        ~observable() = default;

        static auto touch_sensor(const std::string& name, const std::string& port, double frequency)
        {
            auto observable = rxcpp::observable<>::create<sensor_touch_t>(
                [=](rxcpp::subscriber<sensor_touch_t> subscriber) {
                    const uint8_t id = port2id(port);
                    bool errReported = false;
                    BrickPi3 brickPi3;
                    brickPi3.detect();
                    brickPi3.set_sensor_type(id, SENSOR_TYPE_TOUCH_NXT);

                    ros::Rate rate(frequency);
                    while (rxros::ok()) {
                        sensor_touch_t touch;
                        int rc = brickPi3.get_sensor(id, &touch);
                        if (rc == 0) {
                            rxros::logging().debug() << "Touch sensor: pressed " << touch.pressed;
                            subscriber.on_next(touch);
                        } else {
                            errReported = true;
                            brickPi3.reset_all();
                            subscriber.on_error(rxros::exception::system_error(errno, "BrickPi3 failed to read Touch sensor '" +  name + "' rc: " + std::to_string(rc)));
                            break;
                        }
                        rate.sleep();
                    }
                    if (!errReported) {
                        brickPi3.reset_all();
                        subscriber.on_completed();
                    }});
            return observable.subscribe_on(rxcpp::synchronize_new_thread());
        }


        static auto color_sensor(const std::string& name, const std::string& port, double frequency)
        {
            auto observable = rxcpp::observable<>::create<sensor_color_t>(
                [=](rxcpp::subscriber<sensor_color_t> subscriber) {
                    const uint8_t id = port2id(port);
                    bool errReported = false;
                    BrickPi3 brickPi3;
                    brickPi3.detect();
                    brickPi3.set_sensor_type(id, SENSOR_TYPE_NXT_COLOR_FULL);

                    ros::Rate rate(frequency);
                    while (rxros::ok()) {
                        sensor_color_t color;
                        int rc = brickPi3.get_sensor(id, &color);
                        if (rc == 0) {
                            rxros::logging().debug() << "Color sensor: " << color.color << ", red: " << color.reflected_red << ", green: " << color.reflected_green << ", blue: " << color.reflected_blue << ", ambient: " << color.ambient;
                            subscriber.on_next(color);
                        } else {
                            errReported = true;
                            brickPi3.reset_all();
                            subscriber.on_error(rxros::exception::system_error(errno, "BrickPi3 failed to read Color sensor '" +  name + "' rc: " + std::to_string(rc)));
                            break;
                        }
                        rate.sleep();
                    }
                    if (!errReported) {
                        brickPi3.reset_all();
                        subscriber.on_completed();
                    }});
            return observable.subscribe_on(rxcpp::synchronize_new_thread());
        }

        static auto ultrasonic_sensor(const std::string& name, const std::string& port, double frequency)
        {
            auto observable = rxcpp::observable<>::create<sensor_ultrasonic_t>(
                [=](rxcpp::subscriber<sensor_ultrasonic_t> subscriber) {
                    const uint8_t id = port2id(port);
                    bool errReported = false;
                    BrickPi3 brickPi3;
                    brickPi3.detect();
                    brickPi3.set_sensor_type(id, SENSOR_TYPE_NXT_ULTRASONIC);

                    ros::Rate rate(frequency);
                    while (rxros::ok()) {
                        sensor_ultrasonic_t ultrasonic;
                        int rc = brickPi3.get_sensor(id, &ultrasonic);
                        if (rc == 0) {
                            rxros::logging().debug() << "Ultrasonic sensor: cm: " << ultrasonic.cm << ", inches: " << ultrasonic.inch;
                            subscriber.on_next(ultrasonic);
                        } else {
                            errReported = true;
                            brickPi3.reset_all();
                            subscriber.on_error(rxros::exception::system_error(errno, "BrickPi3 failed to read Ultrasonic sensor '" +  name + "' rc: " + std::to_string(rc)));
                            break;
                        }
                        rate.sleep();
                    }
                    if (!errReported) {
                        brickPi3.reset_all();
                        subscriber.on_completed();
                    }});
            return observable.subscribe_on(rxcpp::synchronize_new_thread());
        }

        static auto motor(const std::string &name, const std::string &port, double frequency)
        {
            auto observable = rxcpp::observable<>::create<actuator_motor_t>(
                [=](rxcpp::subscriber<actuator_motor_t> subscriber) {
                    const uint8_t id = port2id(port);
                    bool errReported = false;
                    BrickPi3 brickPi3;
                    brickPi3.detect();
                    brickPi3.reset_motor_encoder(id);

                    ros::Rate rate(frequency);
                    while (rxros::ok()) {
                        actuator_motor_t motor{};
                        int rc = brickPi3.get_motor_status(id, motor.motorState, motor.motorPower, motor.motorPosition, motor.motorDPS);
                        if (rc == 0) {
                            rxros::logging().debug() << "Motor name '" << name << "', State: " << motor.motorState << ", Power: " << motor.motorPower << ", Position: " << motor.motorPosition << ", DPS: " << motor.motorDPS;
                            subscriber.on_next(motor);
                        } else {
                            errReported = true;
                            brickPi3.reset_all();
                            subscriber.on_error(rxros::exception::system_error(errno, "BrickPi3 failed to read Motor '" +  name + "' rc: " + std::to_string(rc)));
                            break;
                        }
                        rate.sleep();
                    }
                    if (!errReported) {
                        brickPi3.reset_all();
                        subscriber.on_completed();
                    }});
            return observable.subscribe_on(rxcpp::synchronize_new_thread());
        }
    };// end class observable
} // end namespace brickpi3


namespace brickpi3
{
    namespace operators
    {
        auto set_motor_power(const std::string& port) {
            return [=](auto&& source) {
                const uint8_t id = port2id(port);
                BrickPi3 brickPi3;
                source.subscribe_on(rxcpp::synchronize_new_thread()).subscribe(
                    [&](const auto& jointCommand) {brickPi3.set_motor_power(id, static_cast<int8_t>(jointCommand.effort * 100.0));});
                return source;};}
    } // end namespace operators
} // end namespace brickpi3


#endif //BRICKPI3_BRICKPI3OBSERVABLE_RX_H
