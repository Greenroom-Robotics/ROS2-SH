// Include the header for the conversions

// Include the header for the logger
//#include <is/utils/Log.hpp>

#include <is/sh/ros2/BaseConverter.hpp>

namespace eprosima::is::sh::ros2 {
    static const eprosima::xtypes::StructType parse_type_idl(
            const std::string &msg_cpp_struct_name,
            const std::string &msg_name,
            const std::string &idl) {
        eprosima::xtypes::idl::Context context;
        context.allow_keyword_identifiers = true;
        context.ignore_redefinition = true;
        eprosima::xtypes::idl::parse(idl, context);
        if (!context.success) {
            std::stringstream ss;
            ss << "Failed while parsing type " << msg_cpp_struct_name;
            throw std::runtime_error(ss.str());
        }
        eprosima::xtypes::StructType type(context.module().structure(msg_cpp_struct_name));
        type.name(msg_name);
        return type;
    }
}
