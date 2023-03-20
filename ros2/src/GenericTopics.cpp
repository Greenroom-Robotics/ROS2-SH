#include <is/sh/ros2/Factory.hpp>
#include <is/utils/Log.hpp>
#include <rclcpp/node.hpp>

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

//using ConvertToXtypeCall = std::function<void(const rclcpp::SerializedMessage&, eprosima::xtypes::WritableDynamicDataRef)>;
//using ConvertToROS2Call = std::function<void(const eprosima::xtypes::ReadableDynamicDataRef&, rclcpp::SerializedMessage&)>;

static eprosima::is::utils::Logger logger("is::sh::ROS2");

//==============================================================================
class GenericSubscription final
{


public:

    GenericSubscription(
            rclcpp::Node& node,
            TopicSubscriberSystem::SubscriptionCallback* callback,
            const std::string& topic_name,
            const std::string& topic_type,
            const xtypes::DynamicType& message_type,
            const rclcpp::QoS& qos_profile,
            Factory::RegisterDeserialiseToXtypeFactory* convert_to_xtype)
        : _callback(callback)
        , _message_type(message_type)
        , _topic_name(topic_name)
        , _topic_type(topic_type)
        , _convert_to_xtype(convert_to_xtype)
    {
        auto subscription_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
        subscription_options.ignore_local_publications = true; // Enable ignore_local_publications option

        _subscription = node.create_generic_subscription(
                topic_name, _topic_type, qos_profile,
                [=](const std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    this->subscription_callback(msg);
                },
                subscription_options);
    }

private:

    void subscription_callback(
            const std::shared_ptr<rclcpp::SerializedMessage>& msg)
    {
        logger << utils::Logger::Level::INFO
               << "Receiving message from ROS 2 for topic '"
               << _topic_name << "'" << std::endl;

        xtypes::DynamicData data(_message_type);
        (*_convert_to_xtype)(msg, data);

        logger << utils::Logger::Level::INFO
                << "Received message: [[ " << data << " ]]" << std::endl;

        (*_callback)(data, nullptr);
    }

    // Save the callback that we were given by the is-ros2 plugin
    TopicSubscriberSystem::SubscriptionCallback* _callback;

    const xtypes::DynamicType& _message_type;

    std::string _topic_name;
    std::string _topic_type;

    rclcpp::GenericSubscription::SharedPtr _subscription;
    Factory::RegisterDeserialiseToXtypeFactory* _convert_to_xtype;

};

//==============================================================================
class GenericPublisher final : public virtual is::TopicPublisher
{
public:

    GenericPublisher(
            rclcpp::Node& node,
            const std::string& topic_name,
            const std::string& topic_type,
            const rclcpp::QoS& qos_profile,
            Factory::RegisterSerialiseToROS2Factory* convert_to_ros2)
        : _topic_name(topic_name),
            _topic_type(topic_type),
            _convert_to_ros2(convert_to_ros2)
    {
        _publisher = node.create_generic_publisher(
            topic_name,
            topic_type,
            qos_profile);
    }

    bool publish(
            const xtypes::DynamicData& message) override
    {
        rclcpp::SerializedMessage ros2_msg;
        (*_convert_to_ros2)(message, ros2_msg);

        logger << utils::Logger::Level::INFO
            << "Sending message from Integration Service to ROS 2 for topic '" << _topic_name << "': "
            << "[[ " << message << " ]]" << std::endl;

        _publisher->publish(ros2_msg);
        return true;
    }

private:

    rclcpp::GenericPublisher::SharedPtr _publisher;

    std::string _topic_name;
    std::string _topic_type;

    Factory::RegisterSerialiseToROS2Factory* _convert_to_ros2;

};


} //  namespace @(namespace_variable)
} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima
