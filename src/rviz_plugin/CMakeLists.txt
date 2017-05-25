include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

if(${catkin_VERSION} VERSION_GREATER "0.7.0")
find_package(Qt5Core)
find_package(Qt5Gui)

qt5_wrap_cpp(MOC_FILES
  ork_object_display.h
)
else()
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
qt4_wrap_cpp(MOC_FILES
  ork_object_display.h
)
endif()

set(SOURCE_FILES
  ork_object_display.cpp
  ork_object_visual.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                           ${QT_LIBRARIES}
)
if(${catkin_VERSION} VERSION_GREATER "0.7.0")
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui)
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
