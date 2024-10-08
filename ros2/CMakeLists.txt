# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
#  library and integration-service executable CMake project

##################################################################################
# CMake build rules for the Integration Service ROS 2 SystemHandle library
##################################################################################
cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

project(is-ros2 VERSION "3.1.0" LANGUAGES CXX)

include(GNUInstallDirs)


string(REPLACE "/${CMAKE_LIBRARY_ARCHITECTURE}" "" CMAKE_INSTALL_LIBDIR_ARCHIND "${CMAKE_INSTALL_LIBDIR}")

###################################################################################
# Configure options
###################################################################################
option(BUILD_LIBRARY "Compile the ROS 2 SystemHandle" ON)

###################################################################################
# Load external CMake Modules.
###################################################################################
if(BUILD_LIBRARY)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SANITIZERS_ROOT}/cmake)

    find_package(Sanitizers QUIET)

    if(SANITIZE_ADDRESS)
        message(STATUS "Preloading AddressSanitizer library could be done using \"${ASan_WRAPPER}\" to run your program.")
    endif()
endif()

###################################################################################
# External dependencies for the Integration Service ROS 2 SystemHandle library
###################################################################################
if(BUILD_LIBRARY)
    # Check if the user specified which ROS 2 version he/she is using. If not, warn and try to automatically get it
    if(NOT IS_ROS2_DISTRO)

        message(WARNING "
            The variable 'IS_ROS2_DISTRO' was not used. You might want to set it to
            specify which ROS 2 version should be used to compile the ROS 2 System Handle.
            By default, a ROS 2 version from the sourced environment will be retrieved automatically.")

            if ("$ENV{ROS_VERSION}" STREQUAL "1")
                message(FATAL_ERROR "
                    A ROS 1 version was sourced last in your build environment.
                    Please use the 'IS_ROS2_VERSION' variable!")
            else()
                set(IS_ROS2_DISTRO $ENV{ROS_DISTRO})
            endif()

    endif()

    message(STATUS "Compiling the ${PROJECT_NAME} System Handle for ROS 2 ${IS_ROS2_DISTRO}...")

    find_package(is-core REQUIRED)
    find_package(rclcpp REQUIRED)
endif()

###################################################################################
# Configure the Integration Service ROS 2 SystemHandle library
###################################################################################
if(BUILD_LIBRARY)
    add_library(${PROJECT_NAME}
        SHARED
            src/Factory.cpp
            src/SystemHandle.cpp
            src/MetaPublisher.cpp
        )

    if (Sanitizers_FOUND)
        add_sanitizers(${PROJECT_NAME})
    endif()

    if($ENV{ROS_DISTRO} STREQUAL "iron")
        add_definitions(-DROS_IRON)
    endif()

    set_target_properties(${PROJECT_NAME} PROPERTIES
        VERSION
            ${PROJECT_VERSION}
        SOVERSION
            ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        CXX_STANDARD
            17
        CXX_STANDARD_REQUIRED
            YES
        )

    target_compile_options(${PROJECT_NAME}
        PRIVATE
            $<$<CXX_COMPILER_ID:GNU>:-pedantic>
            $<$<CXX_COMPILER_ID:GNU>:-fstrict-aliasing>
            $<$<CXX_COMPILER_ID:GNU>:-Wall>
            $<$<CXX_COMPILER_ID:GNU>:-Wextra>
            $<$<CXX_COMPILER_ID:GNU>:-Wcast-align>
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
            $<$<CXX_COMPILER_ID:MSVC>:/wd4700>
            $<$<CXX_COMPILER_ID:MSVC>:/wd4996>
            $<$<CXX_COMPILER_ID:MSVC>:/wd4820>
            $<$<CXX_COMPILER_ID:MSVC>:/wd4255>
            $<$<CXX_COMPILER_ID:MSVC>:/wd4668>
        )

    message(STATUS "Configuring [${PROJECT_NAME}]...")

    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            is::core
            ${rclcpp_LIBRARIES}
        )

    target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
            ${rclcpp_INCLUDE_DIRS}
        )

    is_generate_export_header(ros2)

    if (NOT rosidl_runtime_cpp_FOUND)
        target_compile_definitions(${PROJECT_NAME}
            PUBLIC
                "IS_SH_ROS2__ROSIDL_GENERATOR_CPP"
        )
    endif()
endif()

###################################################################################
# Install the Integration Service ROS 2 SystemHandle library
###################################################################################
if(BUILD_LIBRARY)
    is_install_middleware_plugin(
        MIDDLEWARE
            ros2
        TARGET
            ${PROJECT_NAME}
        DEPENDENCIES
            rclcpp
        EXTENSIONS
            ${CMAKE_CURRENT_LIST_DIR}/cmake/is_ros2_rosidl_mix.cmake
        )

    install(
        DIRECTORY
            ${CMAKE_CURRENT_LIST_DIR}/include/
        DESTINATION
            ${CMAKE_INSTALL_INCLUDEDIR}
    )

###################################################################################
# Install the Integration Service is-ros2-rosidl-mix plugin
###################################################################################
    is_mix_install_extension(
        IDL_TYPE rosidl
        MIDDLEWARE
            ros2
        MESSAGE
            SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.cpp.em"
            HEADERS "${CMAKE_CURRENT_LIST_DIR}/resources/convert__msg.hpp.em"
        SERVICE
            SOURCES "${CMAKE_CURRENT_LIST_DIR}/resources/convert__srv.cpp.em"
    )

    install (
        DIRECTORY
            "${CMAKE_CURRENT_SOURCE_DIR}/scripts"
        DESTINATION
            "${CMAKE_INSTALL_LIBDIR_ARCHIND}/cmake/${PROJECT_NAME}"
    )
endif()

###################################################################################
# Integration Service ROS 2 SystemHandle tests
###################################################################################
if(BUILD_LIBRARY)
    if(BUILD_TESTS OR BUILD_ROS2_TESTS)
        add_subdirectory(test)
    endif()
endif()

###################################################################################
# Integration Service ROS 2 SystemHandle API Reference
###################################################################################
if(BUILD_API_REFERENCE)
    if(NOT BUILD_LIBRARY)
        find_file(IS_DOXYGEN_CONFIG_FILE doxygen-config.in PATHS ${CMAKE_INSTALL_PREFIX}/../is-core)
    endif()
    find_package(Doxygen REQUIRED)
    # Create doxygen directories
    add_custom_target(doc-dirs
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doxygen
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/doxygen/html
        COMMENT "Creating documentation directories" VERBATIM)

    set(IS_ROS2_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include/is/sh/ros2")
    set(IS_ROS2_SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
    file(GLOB_RECURSE HPP_FILES
        "${IS_ROS2_INCLUDE_DIR}/*.h*"
        "${IS_ROS2_SOURCE_DIR}/*.h*")

    # Doxygen related variables
    set(DOXYGEN_INPUT_DIR "${IS_ROS2_INCLUDE_DIR} ${IS_ROS2_SOURCE_DIR}")
    set(DOXYGEN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/doxygen")
    set(DOXYGEN_INDEX_FILE "${PROJECT_BINARY_DIR}/doxygen/xml/index.xml")
    set(DOXYFILE_IN "${IS_DOXYGEN_CONFIG_FILE}")
    set(DOXYFILE_OUT ${PROJECT_BINARY_DIR}/doxygen-config)
    set(DOXYGEN_TAGFILES "../is-core/is_core.tag=../is-core/doxygen/html")
    set(DOXYGEN_HTML_DIR "${PROJECT_BINARY_DIR}/doxygen/html")

    # Configure doxygen
    configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

    # Doxygen command
    add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        DEPENDS ${HPP_FILES}
        MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
        COMMENT "Generating doxygen documentation")

    # Generate API reference
    add_custom_target(doxygen-${PROJECT_NAME} ALL
        DEPENDS ${DOXYGEN_INDEX_FILE}
        COMMENT "Generated API documentation with doxygen" VERBATIM)
    add_dependencies(doxygen-${PROJECT_NAME} doc-dirs)

    # Install doxygen generated XML files
    install(DIRECTORY ${PROJECT_BINARY_DIR}/doxygen/xml
        DESTINATION doxygen
        COMPONENT doxygen-${PROJECT_NAME})
    set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "ROS2 System Handle doxygen")
    set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION
        "eProsima ROS2 System Handle doxygen documentation")
    set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} doxygen-${PROJECT_NAME})
endif()