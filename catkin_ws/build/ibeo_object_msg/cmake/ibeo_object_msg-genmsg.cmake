# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ibeo_object_msg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iibeo_object_msg:/constraint_model/catkin_ws/src/ibeo_object_msg/msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Ishape_msgs:/opt/ros/melodic/share/shape_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ibeo_object_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg" NAME_WE)
add_custom_target(_ibeo_object_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ibeo_object_msg" "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg" "shape_msgs/SolidPrimitive:geometry_msgs/Twist:geometry_msgs/PoseWithCovariance:geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/TwistWithCovariance:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ibeo_object_msg
  "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ibeo_object_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(ibeo_object_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ibeo_object_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ibeo_object_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ibeo_object_msg_generate_messages ibeo_object_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg" NAME_WE)
add_dependencies(ibeo_object_msg_generate_messages_cpp _ibeo_object_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ibeo_object_msg_gencpp)
add_dependencies(ibeo_object_msg_gencpp ibeo_object_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ibeo_object_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ibeo_object_msg
  "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ibeo_object_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(ibeo_object_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ibeo_object_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ibeo_object_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ibeo_object_msg_generate_messages ibeo_object_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg" NAME_WE)
add_dependencies(ibeo_object_msg_generate_messages_eus _ibeo_object_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ibeo_object_msg_geneus)
add_dependencies(ibeo_object_msg_geneus ibeo_object_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ibeo_object_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ibeo_object_msg
  "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ibeo_object_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(ibeo_object_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ibeo_object_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ibeo_object_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ibeo_object_msg_generate_messages ibeo_object_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg" NAME_WE)
add_dependencies(ibeo_object_msg_generate_messages_lisp _ibeo_object_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ibeo_object_msg_genlisp)
add_dependencies(ibeo_object_msg_genlisp ibeo_object_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ibeo_object_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ibeo_object_msg
  "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ibeo_object_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ibeo_object_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ibeo_object_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ibeo_object_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ibeo_object_msg_generate_messages ibeo_object_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg" NAME_WE)
add_dependencies(ibeo_object_msg_generate_messages_nodejs _ibeo_object_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ibeo_object_msg_gennodejs)
add_dependencies(ibeo_object_msg_gennodejs ibeo_object_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ibeo_object_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ibeo_object_msg
  "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/shape_msgs/cmake/../msg/SolidPrimitive.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ibeo_object_msg
)

### Generating Services

### Generating Module File
_generate_module_py(ibeo_object_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ibeo_object_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ibeo_object_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ibeo_object_msg_generate_messages ibeo_object_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/constraint_model/catkin_ws/src/ibeo_object_msg/msg/IbeoObject.msg" NAME_WE)
add_dependencies(ibeo_object_msg_generate_messages_py _ibeo_object_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ibeo_object_msg_genpy)
add_dependencies(ibeo_object_msg_genpy ibeo_object_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ibeo_object_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ibeo_object_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ibeo_object_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(ibeo_object_msg_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ibeo_object_msg_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ibeo_object_msg_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET shape_msgs_generate_messages_cpp)
  add_dependencies(ibeo_object_msg_generate_messages_cpp shape_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ibeo_object_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ibeo_object_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ibeo_object_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(ibeo_object_msg_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ibeo_object_msg_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ibeo_object_msg_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET shape_msgs_generate_messages_eus)
  add_dependencies(ibeo_object_msg_generate_messages_eus shape_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ibeo_object_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ibeo_object_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ibeo_object_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(ibeo_object_msg_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ibeo_object_msg_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ibeo_object_msg_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET shape_msgs_generate_messages_lisp)
  add_dependencies(ibeo_object_msg_generate_messages_lisp shape_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ibeo_object_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ibeo_object_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ibeo_object_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(ibeo_object_msg_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ibeo_object_msg_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ibeo_object_msg_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET shape_msgs_generate_messages_nodejs)
  add_dependencies(ibeo_object_msg_generate_messages_nodejs shape_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ibeo_object_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ibeo_object_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ibeo_object_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ibeo_object_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(ibeo_object_msg_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ibeo_object_msg_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ibeo_object_msg_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET shape_msgs_generate_messages_py)
  add_dependencies(ibeo_object_msg_generate_messages_py shape_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ibeo_object_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
