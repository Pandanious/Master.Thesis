cmake_minimum_required(VERSION 3.5.1)
project(gazebo_sim_walk)

set(CMAKE_CXX_STANDARD 17)

SET(DISABLE_CLANG_TIDY FALSE CACHE BOOL "Disable clang-tidy checks")
#if(NOT DISABLE_CLANG_TIDY)
#  find_program(TIDY clang-tidy)
#  if(NOT TIDY)
#      message(FATAL_ERROR "clang-tidy not found!
#      run 
#        apt-install clang clang-tidy clang-tools
#      to install or set DISABLE_CLANG_TIDY")
#  endif()
#  set(CMAKE_CXX_CLANG_TIDY clang-tidy; -warnings-as-errors=*;)
#endif()




include_directories(include)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_plugins
  gazebo_ros
  roscpp
)
#link_directories(${catkin_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
        LIBRARIES
        CATKIN_DEPENDS roscpp
        DEPENDS
)

## Plugins
add_library(gazebo_sim_walk
  src/gazebo_sim_walk.cpp
  src/task_config.cpp
 # src/task_setup.cpp
  #src/cml_randomizer.cpp
)
target_link_libraries(gazebo_sim_walk ${catkin_LIBRARIES})

add_library(gazebo_sim_operator_plugin
  src/operator.cpp
)
target_link_libraries(gazebo_sim_operator_plugin ${catkin_LIBRARIES})

#add_library(gazebo_sim_gui
#  ${headers_MOC}
#  src/gazebo_sim_gui.cc
#)
#target_link_libraries(gazebo_sim_gui ${catkin_LIBRARIES})
#target_link_libraries(gazebo_sim_gui
#    ${Qt5Core_LIBRARIES}
#    ${Qt5Widgets_LIBRARIES}
##    ${PROTOBUF_LIBRARIES}
#    ${Boost_LIBRARIES}
#)


#add_library(Walk
#  src/random_walker.cpp
#)
#target_link_libraries(rcsim_carry_my_luggage_walker_plugin ${catkin_LIBRARIES})#

# Install Gazebo System Plugins
#install (TARGETS gazebo_sim_walk gazebo_sim_operator_plugin gazebo_sim_gui
install (TARGETS gazebo_sim_walk gazebo_sim_operator_plugin 

#rcsim_carry_my_luggage_operator_plugin gazebo_sim_walk rcsim_carry_my_luggage_walker_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY res
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
