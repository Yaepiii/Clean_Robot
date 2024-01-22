cmake_minimum_required(VERSION 3.0.2)
project(show_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    tf
    visualization_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(show_trajectory src/show_trajectory.cpp)
add_dependencies(show_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_trajectory ${catkin_LIBRARIES})


#############
## Install ##
#############


#############
## Testing ##
#############

