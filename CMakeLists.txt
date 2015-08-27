cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

include_directories( ${QT_INCLUDE_DIRS} )
link_directories( ${QT_LIBRARY_DIRS} )

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  include/label_tool.h
)

set(SOURCE_FILES
  src/label_tool.cpp
  ${MOC_FILES}
)

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(OGRE OGRE)
#include_directories( ${OGRE_INCLUDE_DIRS} )
#link_directories( ${OGRE_LIBRARY_DIRS} )

rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})
#rosbuild_link_boost(rviz_spatial_db_plugin system)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

#rosbuild_add_library(rviz_polygon_tool src/polygon_tool.cpp)
#rosbuild_link_boost(rviz_polygon_tool system)
#target_link_libraries(rviz_polygon_tool ${OGRE_LIBRARIES} ${QT_LIBRARIES})

#rosbuild_add_library(rviz_label_tool src/label_tool.cpp)
#rosbuild_link_boost(rviz_label_tool system)
