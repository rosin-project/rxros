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

#ifndef RXROS_TF_INCLUDE_RXROS_TF_H_
#define RXROS_TF_INCLUDE_RXROS_TF_H_

#include <rxros/rxros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


namespace rxros
{
    namespace observable
    {
        static auto from_transform(const std::string& parent_frameId, const std::string& child_frameId, const double frequencyInHz = 10.0)
        {
            auto observable = rxcpp::observable<>::create<tf::StampedTransform>(
                [=](rxcpp::subscriber<tf::StampedTransform> subscriber) {
                    tf::TransformListener transform_listener;
                    ros::Rate rate(frequencyInHz);
                    bool errReported = false;
                    while (rxros::ok()) {
                        try {
                            tf::StampedTransform transform;
                            transform_listener.lookupTransform(parent_frameId, child_frameId, ros::Time(0), transform);
                            subscriber.on_next(transform);
                        }
                        catch (...) {
                            std::exception_ptr err = std::current_exception();
                            subscriber.on_error(err);
                            errReported = true;
                            break;
                        }
                        rate.sleep();
                    }
                    if (!errReported) {
                        subscriber.on_completed();
                    }});
            return observable.subscribe_on(rxcpp::synchronize_new_thread());
        }
    } // end of namespace observable
} // end of namespace rxros


namespace rxros
{
    namespace operators
    {
        auto send_transform() {
            return [=](auto&& source) {
                tf::TransformBroadcaster transformBroadcaster;
                source.observe_on(rxcpp::synchronize_new_thread()).subscribe(
                    [&](const tf::StampedTransform& stf) {transformBroadcaster.sendTransform(stf);});
                return source;};}


        auto send_transform(const std::string &parent_frameId, const std::string &child_frameId) {
            return [=](auto&& source) {
                tf::TransformBroadcaster transformBroadcaster;
                source.observe_on(rxcpp::synchronize_new_thread()).subscribe(
                    [&](const tf::Transform& tf) {transformBroadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), parent_frameId, child_frameId));});
                return source;};}

    } // end namespace operators
} // end namespace rxros


#endif // RXROS_TF_INCLUDE_RXROS_TF_H_
