cmake_minimum_required(VERSION 2.8.3)
project(multimux)
find_package(catkin COMPONENTS cpp_common message_generation rosconsole roscpp rostest rostime rosunit std_msgs xmlrpcpp topic_tools)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

catkin_python_setup()
catkin_package(
  CATKIN_DEPENDS message_runtime rosconsole roscpp std_msgs xmlrpcpp
)

add_executable(multimux src/multimux.cpp)
target_link_libraries(multimux ${catkin_LIBRARIES})

install(TARGETS multimux
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
