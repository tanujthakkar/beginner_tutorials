cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Count.srv"
 )
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
)

add_library(publisher_lib
  src/publisher.cpp)

ament_target_dependencies(publisher_lib rclcpp std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(publisher_lib "${cpp_typesupport_target}")

add_executable(talker src/publisher.cpp)
target_link_libraries(talker
  publisher_lib
)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/beginner_tutorials
  DESTINATION include
)

# Install shared library
install(
  TARGETS publisher_lib
  EXPORT publisher_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/test.cpp
  )

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test
    publisher_lib
  )

  ament_target_dependencies(
    ${PROJECT_NAME}_test
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  install(
    TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_package()