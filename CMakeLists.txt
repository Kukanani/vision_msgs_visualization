cmake_minimum_required(VERSION 2.8)
project(vision_msgs_visualization)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS pluginlib rviz vision_msgs)
find_package(Boost REQUIRED system filesystem serialization)
catkin_package(
  # DEPENDS
  CATKIN_DEPENDS rviz vision_msgs
  LIBRARIES ${PROJECT_NAME}
)

if(${catkin_VERSION} VERSION_GREATER "0.7.0")
  find_package(Qt5Core)
  find_package(Qt5Gui)

  qt5_wrap_cpp(MOC_FILES
    src/ork_object_display.h
  )
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
  add_definitions(-DQT_NO_KEYWORDS)
  qt4_wrap_cpp(MOC_FILES
    src/ork_object_display.h
  )
endif()


# build some cells
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

set(SOURCE_FILES
  src/ork_object_display.cpp
  src/ork_object_visual.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})
if(${catkin_VERSION} VERSION_GREATER "0.7.0")
  target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui)
endif()



install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
