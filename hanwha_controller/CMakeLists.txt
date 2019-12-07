cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG} -std=c++14 -w")
project(hanwha_controller)
add_definitions(-std=c++14  -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS)

include_directories(
	${catkin_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include/${PROJECT_NAME})
include_directories(include/${PROJECT_NAME})

add_library(hanwha_arm_controller src/hanwha_arm_controller.cpp)
target_link_libraries(hanwha_arm_controller)

add_executable( hanwha_controller_node src/main.cpp )
target_link_libraries(  hanwha_controller_node hanwha_arm_controller ) 

install(TARGETS hanwha_arm_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(DIRECTORY src/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

# install(DIRECTORY src/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
