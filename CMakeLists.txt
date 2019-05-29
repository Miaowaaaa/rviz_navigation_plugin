cmake_minimum_required(VERSION 2.8.3)
project(rviz_navigation_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rviz cmake_modules)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

if (rviz_QT_VERSION VERSION_GREATER "5.0.0")
  find_package(Qt5Core)
  find_package(Qt5Gui)
  find_package(Qt5Widgets)
  catkin_package()
  set (Qt5QtCore_DIR /usr/lib/x86_64-linux-gnu/cmake)
  include_directories( include ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS})

  set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES})

  qt5_wrap_cpp(MOC_FILES
    include/rviz_velocity_panel.h
  )
  add_definitions(-DQT_NO_KEYWORDS)
  set(SOURCE_FILES
    src/rviz_velocity_panel.cpp
    ${MOC_FILES}
  )
  
  ## Declare a C++ library
  add_library(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_FILES})
  target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  catkin_package()
  include(${QT_USE_FILE})

  include_directories( include )

  qt4_wrap_cpp(MOC_FILES
    include/rviz_velocity_panel.h
  )
  add_definitions(-DQT_NO_KEYWORDS)
  set(SOURCE_FILES
    src/rviz_velocity_panel.cpp
    ${MOC_FILES}
  )

  ## Declare a C++ library
  add_library(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_FILES})
  target_link_libraries(${PROJECT_NAME}  ${QT_LIBRARIES}  ${catkin_LIBRARIES})
endif()


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rviz_navigation_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
