########### 版本要求
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

########### 项目名称
project(robot)

########### 添加系统库
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        )

catkin_package()

########### 添加头文件
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
#add_executable(car src/car_node.cpp src/car.cpp)
#target_link_libraries(car ${catkin_LIBRARIES})

add_executable(keyboardcontrol src/keyboardcontrol.cpp)
target_link_libraries(keyboardcontrol ${catkin_LIBRARIES})
