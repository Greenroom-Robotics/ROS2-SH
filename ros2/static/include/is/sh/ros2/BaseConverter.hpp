#ifndef ROS2_SH_BASECONVERTER_HPP
#define ROS2_SH_BASECONVERTER_HPP

#include <is/sh/ros2/Factory.hpp>
#include <is/systemhandle/SystemHandle.hpp>

#include <is/utils/Convert.hpp>

namespace eprosima::is::sh::ros2 {
    static const eprosima::xtypes::StructType parse_type_idl(
            const std::string &msg_cpp_struct_name,
            const std::string &msg_name,
            const std::string &idl);

    class BaseConverter {
    public:
        BaseConverter(
                TopicSubscriberSystem::SubscriptionCallback *callback,
                const std::string &topic_name,
                const xtypes::DynamicType &message_type) :
                _callback(callback), _message_type(message_type), _topic_name(topic_name) {};

//    static const eprosima::xtypes::StructType &get_type();

    protected:
        // Save the callback that we were given by the is-ros2 plugin
        TopicSubscriberSystem::SubscriptionCallback *_callback;
        const xtypes::DynamicType &_message_type;
        std::string _topic_name;
    };
} //  namespace ros2
#endif //ROS2_SH_BASECONVERTER_HPP
