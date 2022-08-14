cmake_minimum_required(VERSION 3.0.2)
project(rviz_rotatable_image_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pluginlib
  roscpp
  rospy
  rviz
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OGRE QUIET COMPONENTS Overlay)
if(OGRE_FOUND)
  if(${OGRE_VERSION} VERSION_LESS 1.11)
    set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${OGRE_Overlay_INCLUDE_DIRS})
    set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${OGRE_Overlay_LIBRARIES})
  endif()
else()
  # OGRE doesn't come with correctly installed cmake files on Linux. Use pkg-config instead.
  # We need both, OGRE and OGRE-Overlay. Look for both simulatenously and use prefix X_
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(X REQUIRED OGRE OGRE-Overlay)

  # Set OGRE_* variables as cmake-based find_package would do
  set(OGRE_VERSION ${X_OGRE_VERSION})
  set(OGRE_INCLUDE_DIRS ${X_INCLUDE_DIRS})
  message(STATUS "OGRE_INCLUDE_DIRS=${OGRE_INCLUDE_DIRS}")

  # Find absolute path of OGRE libraries.
  # This is stored in the cache to save time on cmake re-run.
  # Using absolute paths is necessary if pkg-config finds Ogre in a different
  # location than the default.  This can happen when Ogre is built from source,
  # or when 2 versions of Ogre are installed.  Using absolute paths ensures that
  # components that link against Ogre (rviz and any packages depending on rviz)
  # all find the same Ogre shared library.
  if(NOT DEFINED OGRE_LIBRARIES)
    set(_OGRE_LIBRARIES)
    foreach(_lib ${X_LIBRARIES})
      set(_lib_tag "OGRE_RVIZ_${_lib}")

      find_library(${_lib_tag}
              NAMES ${_lib}
              HINTS ${X_LIBRARY_DIRS}
              PATHS ${X_LIBRARY_DIRS}
      )

      list(APPEND _OGRE_LIBRARIES ${${_lib_tag}})
    endforeach(_lib)

    set(OGRE_LIBRARIES ${_OGRE_LIBRARIES} CACHE STRING "Absolute paths to OGRE libs")
  endif(NOT DEFINED OGRE_LIBRARIES)
  message(STATUS "OGRE_LIBRARIES=${OGRE_LIBRARIES}")

  ## Fetch OGRE_PLUGIN_DIR variable from pkg-config
  if(NOT DEFINED OGRE_PLUGIN_DIR)
    execute_process(COMMAND
      ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
      OUTPUT_VARIABLE OGRE_PLUGIN_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
  endif(NOT DEFINED OGRE_PLUGIN_DIR)
  message(STATUS "OGRE_PLUGIN_DIR=${OGRE_PLUGIN_DIR}")
endif()

# TODO: adapt version after upgrade to newer OGRE release
# Consider deprecation issues as warnings, not as errors for OGRE >= 1.10
if(NOT OGRE_VERSION VERSION_LESS "1.10.0" AND NOT MSVC)
  add_compile_options(-Wno-error=deprecated-declarations)
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include ${OGRE_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
 LIBRARIES ${PROJECT_NAME} ${OGRE_LIBRARIES} ${Qt5Widgets_LIBRARIES}
 CATKIN_DEPENDS cv_bridge pluginlib roscpp rospy rviz sensor_msgs std_msgs
 DEPENDS Boost OGRE Qt5Widgets
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

## Wrapping into MOCS or RViz/Poco complains that the shared object has undefined symbols!
## see https://answers.ros.org/question/206363/rqt-plugin-undefined-symbol/
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rotatable_image_display.h
)
## see https://doc.qt.io/qt-5/qtcore-cmake-qt5-wrap-cpp.html
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rviz_rotatable_image_plugin.cpp
# )

add_library(${PROJECT_NAME}
  plugins/rotatable_image_display.cpp
  ${${PROJECT_NAME}_MOCS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rviz_rotatable_image_plugin_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5Widgets
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rviz_rotatable_image_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
