# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "omcron_package: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(omcron_package_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv" NAME_WE)
add_custom_target(_omcron_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "omcron_package" "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(omcron_package
  "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/omcron_package
)

### Generating Module File
_generate_module_cpp(omcron_package
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/omcron_package
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(omcron_package_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(omcron_package_generate_messages omcron_package_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv" NAME_WE)
add_dependencies(omcron_package_generate_messages_cpp _omcron_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(omcron_package_gencpp)
add_dependencies(omcron_package_gencpp omcron_package_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS omcron_package_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(omcron_package
  "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/omcron_package
)

### Generating Module File
_generate_module_eus(omcron_package
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/omcron_package
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(omcron_package_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(omcron_package_generate_messages omcron_package_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv" NAME_WE)
add_dependencies(omcron_package_generate_messages_eus _omcron_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(omcron_package_geneus)
add_dependencies(omcron_package_geneus omcron_package_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS omcron_package_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(omcron_package
  "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/omcron_package
)

### Generating Module File
_generate_module_lisp(omcron_package
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/omcron_package
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(omcron_package_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(omcron_package_generate_messages omcron_package_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv" NAME_WE)
add_dependencies(omcron_package_generate_messages_lisp _omcron_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(omcron_package_genlisp)
add_dependencies(omcron_package_genlisp omcron_package_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS omcron_package_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(omcron_package
  "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/omcron_package
)

### Generating Module File
_generate_module_nodejs(omcron_package
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/omcron_package
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(omcron_package_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(omcron_package_generate_messages omcron_package_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv" NAME_WE)
add_dependencies(omcron_package_generate_messages_nodejs _omcron_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(omcron_package_gennodejs)
add_dependencies(omcron_package_gennodejs omcron_package_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS omcron_package_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(omcron_package
  "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/omcron_package
)

### Generating Module File
_generate_module_py(omcron_package
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/omcron_package
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(omcron_package_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(omcron_package_generate_messages omcron_package_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/minhtricao/git/RoboSensor/omcron_package/srv/SetPosition.srv" NAME_WE)
add_dependencies(omcron_package_generate_messages_py _omcron_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(omcron_package_genpy)
add_dependencies(omcron_package_genpy omcron_package_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS omcron_package_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/omcron_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/omcron_package
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(omcron_package_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/omcron_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/omcron_package
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(omcron_package_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/omcron_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/omcron_package
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(omcron_package_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/omcron_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/omcron_package
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(omcron_package_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/omcron_package)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/omcron_package\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/omcron_package
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(omcron_package_generate_messages_py std_msgs_generate_messages_py)
endif()
