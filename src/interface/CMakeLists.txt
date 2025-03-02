cmake_minimum_required(VERSION 3.8)
project(interface)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB_RECURSE MSG msg/*.msg)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Armor.msg"
        "msg/Armors.msg"
        "msg/Robot.msg"
        "msg/KalmanPredict.msg"
        "msg/Direction.msg"
        "msg/SaveData.msg"
        "srv/Kalman1.srv"
        "srv/UpdateWriter.srv"
        DEPENDENCIES
        std_msgs
        geometry_msgs
)


ament_package()
