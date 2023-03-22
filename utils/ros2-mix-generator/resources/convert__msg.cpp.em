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

alphabetical_fields = sorted(spec.fields, key=lambda x: x.name)

}@

// Include the API header for this message type
#include <@(conversion_dependency)>


// TODO(jamoralp): Add utils::Logger traces here
namespace eprosima {
namespace is {
namespace sh {
namespace ros2 {
namespace @(namespace_variable) {

    void convert_to_ros2([[maybe_unused]] const eprosima::xtypes::ReadableDynamicDataRef& from, [[maybe_unused]] Ros2_Msg& to)
    {
      @[for field in alphabetical_fields]@
      utils::Convert<Ros2_Msg::_@(field.name)_type>::from_xtype_field(from["@(field.name)"], to.@(field.name));
      @[end for]@
      }

void convert_to_xtype([[maybe_unused]] const Ros2_Msg& from, [[maybe_unused]]eprosima::xtypes::WritableDynamicDataRef to)
    {
      @[for field in alphabetical_fields]@
      utils::Convert<Ros2_Msg::_@(field.name)_type>::to_xtype_field(from.@(field.name), to["@(field.name)"]);
      @[end for]@
      }

    void serialise([[maybe_unused]] const eprosima::xtypes::ReadableDynamicDataRef& from, [[maybe_unused]] rclcpp::SerializedMessage& serialised_msg)
    {
      Ros2_Msg temp_msg;
      convert_to_ros2(from, temp_msg);
      rclcpp::Serialization<Ros2_Msg> ser;
      ser.serialize_message(&temp_msg, &serialised_msg);
    }

    void deserialise([[maybe_unused]] const rclcpp::SerializedMessage& serialised_msg, [[maybe_unused]]eprosima::xtypes::WritableDynamicDataRef to)
    {
      Ros2_Msg from;
      rclcpp::Serialization<Ros2_Msg> serializer;
      serializer.deserialize_message(&serialised_msg, &from);

      convert_to_xtype(from, to);
    }

namespace {
            TypeToFactoryRegistrar register_type(g_msg_name, &type);
            SerialiseToROS2FactoryRegistrar register_serialiser(g_msg_name, &serialise);
            DeserialiseToXtypeFactoryRegistrar register_deserialiser(g_msg_name, &deserialise);
        } // anonymous namespace

} //  namespace @(namespace_variable)
} //  namespace ros2
} //  namespace sh
} //  namespace is
} //  namespace eprosima
