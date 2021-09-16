cmake_minimum_required(VERSION 2.8.3)
project(boost_ros_pkg)
find_package(catkin REQUIRED)

message(MODULE=${MODULE})
# build boost from source
execute_process(COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.boost PREFIX=${CATKIN_DEVEL_PREFIX} MODULE=${MODULE} install.boost)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include) # fake install directory for catkin_package
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  DEPENDS BOOST_PKG
)

