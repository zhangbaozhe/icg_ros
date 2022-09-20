cmake_minimum_required(VERSION 3.0.2)
project(icg_ros)
set(CMAKE_CXX_STANDARD 17)

# ===================== External Dependencies =====================
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)

# ===================== External Dependencies =====================


# ===================== ROS Dependencies =====================

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        cv_bridge
        message_filters)

# ===================== ROS Dependencies =====================


# ===================== Checking =====================

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-march HAS_MARCH)
check_cxx_compiler_flag(-mtune HAS_MTUNE)
if (HAS_MARCH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()
if (HAS_MTUNE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
endif ()
check_cxx_compiler_flag(-ggdb HAS_GGDB)
check_cxx_compiler_flag(-Z7 HAS_Z7)
check_cxx_compiler_flag(-ftrapv HAS_FTRAPV)
check_cxx_compiler_flag(-Od HAS_OD)
if (HAS_GGDB)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")
elseif (HAS_Z7)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Z7")
endif ()
if (HAS_FTRAPV)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftrapv")
endif ()
if (HAS_OD)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Od")
endif ()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

check_cxx_compiler_flag(-Ob3 HAS_OB3)
check_cxx_compiler_flag(-O3 HAS_O3)
check_cxx_compiler_flag(-Ob2 HAS_OB2)
check_cxx_compiler_flag(-O2 HAS_O2)
if (HAS_OB3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob3")
elseif (HAS_O3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif (HAS_OB2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob2")
elseif (HAS_O2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")

# ===================== Checking =====================


# ===================== Set group name =====================

set(SOURCES
        src/icg/common.cpp
        src/icg/body.cpp
        src/icg/renderer_geometry.cpp
        src/icg/renderer.cpp
        src/icg/normal_renderer.cpp
        src/icg/basic_depth_renderer.cpp
        src/icg/model.cpp
        src/icg/region_model.cpp
        src/icg/depth_model.cpp
        src/icg/camera.cpp
        src/icg/loader_camera.cpp
        src/icg/viewer.cpp
        src/icg/image_viewer.cpp
        src/icg/normal_viewer.cpp
        src/icg/modality.cpp
        src/icg/region_modality.cpp
        src/icg/depth_modality.cpp
        src/icg/optimizer.cpp
        src/icg/detector.cpp
        src/icg/static_detector.cpp
        src/icg/manual_detector.cpp
        src/icg/refiner.cpp
        src/icg/publisher.cpp
        src/icg/tracker.cpp
        src/icg/generator.cpp src/icg_ros/ros_camera.cpp)

set(LIBRARIES
        Eigen3::Eigen
        OpenGL::GL
        GLEW::GLEW
        glfw
        ${OpenCV_LIBRARIES})

if(UNIX)
    list(APPEND LIBRARIES pthread stdc++fs)
endif()

# ===================== Set group name =====================


# ===================== STATIC Library =====================

catkin_package(
        INCLUDE_DIRS include third_party
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rospy std_msgs)

include_directories(
        include
        third_party
        ${catkin_INCLUDE_DIRS})

add_library(libicg
        STATIC ${SOURCES})
# ===================== STATIC Library =====================


# ===================== Executables =====================

add_executable(icg_test_node
        src/icg_test_node.cpp src/icg_ros/ros_camera.cpp)

# ===================== Executables =====================

# ===================== Linking =====================

target_link_libraries(icg_test_node libicg ${catkin_LIBRARIES})

target_link_libraries(libicg PUBLIC
        ${LIBRARIES})

install(TARGETS libicg
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(
        DIRECTORY include/icg/
        DIRECTORY third_party/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
# ===================== Linking =====================

