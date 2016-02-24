cmake_minimum_required(VERSION 2.8.3)
project(rviz_semap_plugin)

set(THIS_PACKAGE_ROS_DEPS
  rospy
  roscpp
  std_msgs
  mesh_msgs
  geometry_msgs
  visualization_msgs
  rviz
  interactive_markers
  lvr_ros
  semap
  semap_ros
  semap_env
  python_qt_binding)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(catkin REQUIRED COMPONENTS 
  ${THIS_PACKAGE_ROS_DEPS} 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${QT_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS Qt4
)

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  include/label_viz.h
  include/label_tool.h
  include/semap_panel.h
  include/semap_widgets.h
)


set(SOURCE_FILES
  src/label_viz.cpp
  src/label_tool.cpp
  src/semap_panel.cpp
  src/semap_widgets.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(polygon_client src/polygon_client.cpp src/polygon_client.cpp)
target_link_libraries(polygon_client ${QT_LIBRARIES}  ${catkin_LIBRARIES})
#add_dependencies(polygon_client ${catkin_EXPORTED_TARGETS})

install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(OGRE OGRE)
#include_directories( ${OGRE_INCLUDE_DIRS} )
#link_directories( ${OGRE_LIBRARY_DIRS} )
#rosbuild_link_boost(rviz_spatial_db_plugin system)
#rosbuild_add_library(rviz_polygon_tool src/polygon_tool.cpp)
#rosbuild_link_boost(rviz_polygon_tool system)
#target_link_libraries(rviz_polygon_tool ${OGRE_LIBRARIES} ${QT_LIBRARIES})
#rosbuild_link_boost(rviz_label_tool system)
