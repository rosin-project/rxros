cmake_minimum_required(VERSION 2.8.3)
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rxros
  std_msgs
  teleop_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rxros
    std_msgs
    teleop_msgs
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(joystick_publisher       src/JoystickPublisher.cpp)
add_executable(joystick_publisher_rxros src/JoystickPublisher_rx.cpp)
add_executable(keyboard_publisher       src/KeyboardPublisher.cpp)
add_executable(keyboard_publisher_rxros src/KeyboardPublisher_rx.cpp)
add_executable(velocity_publisher       src/VelocityPublisher.cpp)
add_executable(velocity_publisher_rxros src/VelocityPublisher_rx.cpp)

add_dependencies(joystick_publisher       ${catkin_EXPORTED_TARGETS})
add_dependencies(joystick_publisher_rxros ${catkin_EXPORTED_TARGETS})
add_dependencies(keyboard_publisher       ${catkin_EXPORTED_TARGETS})
add_dependencies(keyboard_publisher_rxros ${catkin_EXPORTED_TARGETS})
add_dependencies(velocity_publisher       ${catkin_EXPORTED_TARGETS})
add_dependencies(velocity_publisher_rxros ${catkin_EXPORTED_TARGETS})

target_link_libraries(joystick_publisher       ${catkin_LIBRARIES})
target_link_libraries(joystick_publisher_rxros ${catkin_LIBRARIES})
target_link_libraries(keyboard_publisher       ${catkin_LIBRARIES})
target_link_libraries(keyboard_publisher_rxros ${catkin_LIBRARIES})
target_link_libraries(velocity_publisher       ${catkin_LIBRARIES})
target_link_libraries(velocity_publisher_rxros ${catkin_LIBRARIES})
