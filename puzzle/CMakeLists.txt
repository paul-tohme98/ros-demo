cmake_minimum_required(VERSION 3.0.2)
project(puzzle)

set(CMAKE_CXX_STANDARD 17)

# set(Python3_ROOT_DIR "/usr/bin/python3")
# list(APPEND CMAKE_PREFIX_PATH "/usr/include/python3/") # Remplacez x par la version de Python3 installée sur votre machine

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  nb_pieces.msg
  pieces_info.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

catkin_package(
 # LIBRARIES puzzle
 CATKIN_DEPENDS roscpp std_msgs message_runtime cv_bridge sensor_msgs
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${cv_bridge_INCLUDE_DIRS})

add_executable(pieces_info_pub src/pieces_info_pub.cpp)
add_dependencies(pieces_info_pub ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pieces_info_pub ${catkin_LIBRARIES})
target_link_libraries(pieces_info_pub ${OpenCV_LIBRARIES})
# target_link_libraries(pieces_info_pub ${cv_bridge_LIBRARIES})

add_executable(pieces_info_sub src/pieces_info_sub.cpp)
add_dependencies(pieces_info_sub ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pieces_info_sub ${catkin_LIBRARIES})
# target_link_libraries(pieces_info_sub ${cv_bridge_LIBRARIES})
# target_link_libraries(pieces_info_sub ${OpenCV_LIBS})
