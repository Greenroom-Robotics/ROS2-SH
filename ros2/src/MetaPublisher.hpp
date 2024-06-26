/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef _IS_SH_ROS2__INTERNAL__METAPUBLISHER_HPP_
#define _IS_SH_ROS2__INTERNAL__METAPUBLISHER_HPP_

#include <is/systemhandle/SystemHandle.hpp>

#include <rclcpp/node.hpp>

namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {

//==============================================================================
/**
 * @brief Produces a is::TopicPublisher that allows to use runtime substitution parameters
 * in the *YAML* configuration file.
 *
 * @see is::core::StringTemplate
 *
 * @param[in] message_type A reference to the dynamic type representation of the topic type.
 *
 * @param[in] node The ROS 1 node that will hold this publisher.
 *
 * @param[in] topic_name The topic name to publish to.
 *
 * @param[in] qos_profile The QoS used to create the publisher.
 *
 * @param[in] configuration The configuration specific for this SystemHandle,
 *            as described in the user-provided *YAML* input file.
 *
 * @returns A pointer to the created *Integration Service* TopicPublisher entity.
 */
std::shared_ptr<TopicPublisher> make_meta_publisher(
        const std::string& topic_name,
        const eprosima::xtypes::DynamicType& message_type,
        rclcpp::Node& node,
        const rclcpp::QoS& qos_profile,
        const YAML::Node& configuration);

std::shared_ptr<void> make_meta_subscriber(
        const std::string& topic_name,
        const eprosima::xtypes::DynamicType& message_type,
        TopicSubscriberSystem::SubscriptionCallback* callback,
        rclcpp::Node& node,
        const rclcpp::QoS& qos_profile,
        const YAML::Node& configuration);

} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima

#endif //  _IS_SH_ROS2__INTERNAL__METAPUBLISHER_HPP_
