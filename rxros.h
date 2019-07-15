//
// Created by hl on 2/24/19.
//

#ifndef RXROS_H
#define RXROS_H

#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cassert>
#include <mutex>
#include <linux/input.h>
#include <rxcpp/rx.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


namespace rxros
{
    static void init(int argc, char** argv, const std::string& name) {
        ros::init(argc, argv, name);
    }

    static void spin() {
        ros::MultiThreadedSpinner spinner(0); // Use a spinner for each available CPU core
        spinner.spin();
    }

    static bool ok() {
        return ros::ok();
    }


    class node: public ros::NodeHandle
    {
    private:
        node() = default;

    public:
        ~node() = default;

        // subscribe and advertiseService are overloaded ros::Nodehandle functions for subscribing to a topic and service.
        // The special about these two functions is that they allow the callback to be a std::functions.
        // This means that it will be possible to subscribe to a topic using a lambda expression as a callback.
        // Ideas borrowed from https://github.com/OTL/roscpp14
        template<class T>
        ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size, const std::function<void(const T&)>& callback) {
            return ros::NodeHandle::subscribe<T>(topic, queue_size, static_cast<boost::function<void(const T&)>>(callback));
        }

        template<class T>
        ros::ServiceServer advertiseService(const std::string& service, const std::function<bool(typename T::Request&, typename T::Response&)> callback) {
            return ros::NodeHandle::advertiseService<typename T::Request, typename T::Response>(service, static_cast<boost::function<bool(typename T::Request&, typename T::Response&)>>(callback));
        }

        static auto get_handle() {
            static node self;
            return self;
        }
    }; // end of class node


    class exception
    {
    private:
        exception() = default;

    public:
        ~exception() = default;
        static auto system_error(const int errCode, const std::string &msg) {
            return std::make_exception_ptr(std::system_error(std::error_code(errCode, std::generic_category()), msg));
        }
    }; // end of class exception


    class logging: public std::ostringstream
    {
    private:
        enum LogLevel {DEBUG, INFO, WARN, ERROR, FATAL};
        LogLevel logLevel;

    public:
        logging() = default;
        ~logging() override {
            switch(logLevel) {
                case DEBUG:
                    ROS_DEBUG("%s", str().c_str());
                    break;
                case INFO:
                    ROS_INFO("%s", str().c_str());
                    break;
                case WARN:
                    ROS_WARN("%s", str().c_str());
                    break;
                case ERROR:
                    ROS_ERROR("%s", str().c_str());
                    break;
                case FATAL:
                    ROS_FATAL("%s", str().c_str());
                    break;
                default:
                    ROS_FATAL("Ups!!!!");
                    break;
            }
        }

        logging& debug() {
            logLevel = DEBUG;
            return *this;
        }

        logging& info() {
            logLevel = INFO;
            return *this;
        }

        logging& warn() {
            logLevel = WARN;
            return *this;
        }

        logging& error() {
            logLevel = ERROR;
            return *this;
        }

        logging& fatal() {
            logLevel = FATAL;
            return *this;
        }
    }; // end of class logging


    class parameter
    {
    private:
        parameter() = default;

    public:
        ~parameter() = default;

        template<typename T>
        static auto get(const std::string& name, const T& default_value) {
            T param;
            node::get_handle().param<T>(name, param, default_value);
            return param;
        }

        static auto get(const std::string& name, const int default_value) {
            int param;
            node::get_handle().param(name, param, default_value);
            return param;
        }

        static auto get(const std::string& name, const double default_value) {
            double param;
            node::get_handle().param(name, param, default_value);
            return param;
        }

        static auto get(const std::string& name, const char* default_value) {
            return get<std::string>(name, default_value);
        }

        static auto get(const std::string& name, const std::string& default_value) {
            return get<std::string>(name, default_value);
        }
    }; // end of class parameter


    class observable
    {
    private:
        observable() = default;

    public:
        ~observable() = default;

        template<class T>
        static auto from_topic(const std::string& topic, const uint32_t queueSize = 10)
        {
            auto observable = rxcpp::observable<>::create<T>(
                [=](rxcpp::subscriber<T> subscriber) {
                    auto callback = [=](const T &val) {
                        static std::mutex mutex;
                        std::lock_guard<std::mutex> guard(mutex);
                        subscriber.on_next(val);};
                    ros::Subscriber ros_subscriber(node::get_handle().subscribe<T>(topic, queueSize, callback));
                    ros::waitForShutdown();});
            return observable.subscribe_on(rxcpp::synchronize_new_thread());
        }

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

        template<class T>
        static auto from_device(const std::string& device_name)
        {
            auto observable = rxcpp::observable<>::create<T>(
                [=](rxcpp::subscriber<T> subscriber) {
                    int fd = open(device_name.c_str(), O_RDONLY | O_NONBLOCK);
                    if (fd < 0)
                        subscriber.on_error(rxros::exception::system_error(errno, std::string("Cannot open device ") + device_name));
                    else {
                        fd_set readfds; // initialize file descriptor set.
                        FD_ZERO(&readfds);
                        FD_SET(fd, &readfds);
                        T event{};
                        bool errReported = false;
                        while (rxros::ok()) {
                            int rc = select(fd + 1, &readfds, nullptr, nullptr, nullptr);  // wait for input on keyboard device
                            if (rc == -1 && errno == EINTR) { // select was interrupted. This is not an error but we exit the loop
                                subscriber.on_completed();
                                close(fd);
                                break;
                            }
                            else if (rc == -1 || rc == 0) { // select failed and we issue an error.
                                subscriber.on_error(rxros::exception::system_error(errno, std::string("Failed to read device ") + device_name));
                                close(fd);
                                errReported = true;
                                break;
                            }
                            else if (FD_ISSET(fd, &readfds)) {
                                ssize_t sz = read(fd, &event, sizeof(T)); // read element from device.
                                if (sz == -1) {
                                    subscriber.on_error(rxros::exception::system_error(errno, std::string("Failed to read device ") + device_name));
                                    close(fd);
                                    errReported = true;
                                    break;
                                }
                                else if (sz == sizeof(T)) {
                                    subscriber.on_next(event); // populate the event on the
                                }
                            }
                        }
                        if (!errReported) {
                            subscriber.on_completed();
                        }
                    }
                });
            return observable.subscribe_on(rxcpp::synchronize_new_thread());
        }

        // Parse the robot.yaml file and create an observable stream for the configuration of the sensors and actuators
        static auto from_yaml(const std::string& aNamespace)
        {
            return rxcpp::observable<>::create<XmlRpc::XmlRpcValue>(
                [=](rxcpp::subscriber<XmlRpc::XmlRpcValue> subscriber) {
                    XmlRpc::XmlRpcValue robot_config;
                    node::get_handle().getParam(aNamespace, robot_config);
                    assert (robot_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
                    for (int i = 0; i < robot_config.size(); i++) {
                        XmlRpc::XmlRpcValue device_config(robot_config[i]);
                        subscriber.on_next(device_config);
                    }
                    subscriber.on_completed();
                });
        }
    }; // end of class observable
} // end of namespace rxros


namespace rxros
{
    namespace operators
    {
        auto sample_with_frequency(const double frequency) {
            return [=](auto&& source) {
                const std::chrono::milliseconds durationInMs(static_cast<long>(1000.0/frequency));
                return rxcpp::observable<>::interval(durationInMs).with_latest_from(
                    [=](const auto intervalObsrv, const auto sourceObsrv) { return sourceObsrv; }, source);};}


        template<class Coordination>
        auto sample_with_frequency(const double frequency, Coordination coordination) {
            return [=](auto&& source) {
                const std::chrono::milliseconds durationInMs(static_cast<long>(1000.0/frequency));
                return rxcpp::observable<>::interval(durationInMs, coordination).with_latest_from(
                    [=](const auto intervalObsrv, const auto sourceObsrv) { return sourceObsrv; }, source);};}


        template<typename T>
        auto publish_to_topic(const std::string &topic, const uint32_t queue_size = 10) {
            return [=](auto&& source) {
                ros::Publisher publisher(rxros::node::get_handle().advertise<T>(topic, queue_size));
                source.observe_on(rxcpp::synchronize_new_thread()).subscribe(
                    [=](const T& msg) {publisher.publish(msg);});
                return source;};}


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


        template<typename T>
        auto call_service(const std::string& service) {
            return [=](auto&& source) {
                return rxcpp::observable<>::create<T>(
                    [=](rxcpp::subscriber<T> subscriber) {
                        ros::ServiceClient service_client(rxros::node::get_handle().serviceClient<T>(service));
                        source.subscribe(
                            [=](const T& msg) {
                                T res = std::move(msg);
                                if (service_client.call(res))
                                    subscriber.on_next(res);},
                            [=](const std::exception_ptr error){subscriber.on_error(error);},
                            [=](){subscriber.on_completed();});});};}
    } // end namespace operators
} // end namespace rxros


#endif //RXROS_H
