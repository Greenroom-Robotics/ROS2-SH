// generated from is-ros2/resources/convert_msg.cpp.em
// generated code does not contain a copyright notice

@#######################################################################
@# EmPy template for generating is/rosidl/ros2/<package>/src/msg/convert__msg__<msg>.cpp files
@#
@# Context:
@#  - spec (rosidl_adapter.parser.MessageSpecification)
@#    Parsed specification of the .msg/.idl file
@#  - subfolder (string)
@#    The subfolder / subnamespace of the message
@#    Either 'msg' or 'srv'
@#  - get_header_filename_from_msg_name (function)
@#######################################################################

@{
camelcase_msg_type = spec.base_type.type
underscore_msg_type = get_header_filename_from_msg_name(camelcase_msg_type)

namespace_parts = [
    'convert', spec.base_type.pkg_name, 'msg', underscore_msg_type]
namespace_variable = '__'.join(namespace_parts)

conversion_dependency = 'is/rosidl/ros2/{}/msg/convert__msg__{}.hpp'.format(
    spec.base_type.pkg_name, camelcase_msg_type)
}@

// Include the API header for this message type
#include <@(conversion_dependency)>
// Include the Factory header so we can add this message type to the Factory
#include <is/sh/ros2/Factory.hpp>

// Include the Node API so we can subscribe and advertise
#include <rclcpp/node.hpp>

// TODO(jamoralp): Add utils::Logger traces here
namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {
namespace @(namespace_variable) {

//==============================================================================
namespace {
TypeToFactoryRegistrar register_type(g_msg_name, &type);
} // anonymous namespace

//==============================================================================

//==============================================================================
std::shared_ptr<void> subscribe(
        rclcpp::Node& node,
        const std::string& topic_name,
        const xtypes::DynamicType& message_type,
        TopicSubscriberSystem::SubscriptionCallback* callback,
        const rclcpp::QoS& qos_profile)
{
    return std::make_shared<Subscription>(
        node, callback, topic_name, message_type, qos_profile);
}

namespace {
SubscriptionToFactoryRegistrar register_subscriber(g_msg_name, &subscribe);
} // anonymous namespace

//==============================================================================

//==============================================================================
std::shared_ptr<is::TopicPublisher> make_publisher(
        rclcpp::Node& node,
        const std::string& topic_name,
        const rclcpp::QoS& qos_profile)
{
    return std::make_shared<Publisher>(node, topic_name, qos_profile);
}

namespace {
PublisherToFactoryRegistrar register_publisher(g_msg_name, &make_publisher);
} // namespace {

} //  namespace @(namespace_variable)
} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima
