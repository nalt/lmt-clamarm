cmake_minimum_required(VERSION 2.8.3)
project(lmt_clamarm)

set(MOVEIT_PLUGIN_PKG lmt_clamarm_moveit_ikfast)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  rospy
  tf_conversions
  ${MOVEIT_PLUGIN_PKG}
)

message("CatkinDeps: " ${CATKIN_DEPS})

find_package(Boost COMPONENTS python)
find_package(PythonLibs REQUIRED)
#find_package(${MOVEIT_PLUGIN_PKG} REQUIRED)


include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# HACK:
include_directories(../lmt_clamarm_moveit_ikfast/src/)
include_directories(../lmt_clamarm_moveit_ikfast/include/)

link_directories(${catkin_LIBRARY_DIRS})
# add_definitions(-std=gnu++0x)

catkin_python_setup()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

message("PyhtonDir: " ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})


# find_package(LAPACK REQUIRED)

add_library(kinematics_py src/kinematics_base_py.cpp)
target_link_libraries(kinematics_py ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

set_target_properties(kinematics_py PROPERTIES OUTPUT_NAME kinematics_py PREFIX "" SUFFIX ".so")
set_target_properties(kinematics_py PROPERTIES COMPILE_FLAGS "-g -Wno-missing-field-initializers")
set_target_properties(kinematics_py PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

#install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/_kinematics_base_py.so
#  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#)
