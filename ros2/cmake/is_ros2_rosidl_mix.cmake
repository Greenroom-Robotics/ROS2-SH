# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include(CMakeParseArguments)
include(GNUInstallDirs)

#################################################
# is_ros2_rosidl_mix(
#   PACKAGES rosidl_packages...
#   MIDDLEWARES [ros2|websocket|hl7]...
#   [QUIET]
#   [REQUIRED]
# )
#
# Generate an Integration Service middleware interface extension for a set of rosidl packages.
#
# ROS2 packages will often contain message and service specifications in the
# form of rosidl files (.msg and .srv). This cmake utility will convert the
# messages and services into Integration Service middleware interface extension libraries.
# That will allow the Integration Service to pass those message and service types between two
# different middlewares.
#
# The PACKAGES argument specifies the packages whose message and service
# specifications you want to convert (you can specify any number of packages).
# If any of the messages or services in one of the requested packages depends on
# messages in another package, then those package dependencies will be searched
# for to see if they've already been generated. If any of the dependencies have
# not been generated already, then they will be generated by this function call.
#
# The MIDDLEWARES argument specifies which middlewares to create extensions for.
#
# Use the QUIET option to suppress status updates.
#
# Use the REQUIRED option to have a fatal error if anything has prevented the
# mix libraries from being generated. If REQUIRED is not specified, then this
# function will instead print warnings and proceed as much as possible whenever
# an error is encountered.

find_package(is-core REQUIRED)
find_package(rosidl_parser REQUIRED)
find_package(is-ros2 REQUIRED)


set(IS_ROS2_EXTENSION_DIR ${CMAKE_CURRENT_LIST_DIR})

function(is_ros2_rosidl_mix)

    set(possible_options QUIET REQUIRED)

    cmake_parse_arguments(
        _ARG # prefix
        "${possible_options}" # options
        "" # one-value arguments
        "PACKAGES;MIDDLEWARES" # multi-value arguments
        ${ARGN}
    )

    set(options)
    foreach(op ${possible_options})
        if(_ARG_${op})
            list(APPEND options ${op})
        endif()
    endforeach()

    if(NOT Python_EXECUTABLE)
        find_package(Python COMPONENTS Interpreter)
    endif()

    is_mix_generator(
        IDL_TYPE
            rosidl
        SCRIPT
            INTERPRETER
                ${Python_EXECUTABLE}
            FIND
                ${IS_ROS2_EXTENSION_DIR}/../scripts/is_ros2_rosidl_find_package_info.py
            GENERATE
                ${IS_ROS2_EXTENSION_DIR}/../scripts/is_ros2_rosidl_generate.py
        PACKAGES
            ${_ARG_PACKAGES}
        MIDDLEWARES
            ${_ARG_MIDDLEWARES}
        ${options}
    )

endfunction()
