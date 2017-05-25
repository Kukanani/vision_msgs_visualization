cmake_minimum_required(VERSION 2.8)
project(vision_msgs_visualization)

find_package(catkin REQUIRED vision_msgs pluginlib rviz)
catkin_package()

# build some cells
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED system filesystem serialization)

add_subdirectory(src)

# install
install(FILES ork_rviz_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
