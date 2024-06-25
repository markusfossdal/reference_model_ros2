cmake_minimum_required(VERSION 3.8)
project(reference_model_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ref_model_velocity bin/main_velocity.cpp src/ref_model_velocity.cpp)
add_executable(ref_model_position bin/main_position.cpp src/ref_model_position.cpp)

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/reference_model_siso)

ament_target_dependencies(
  ref_model_velocity
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  )

ament_target_dependencies(
  ref_model_position
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  )

target_include_directories(ref_model_velocity PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/reference_model_siso/include
  )

target_include_directories(ref_model_position PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/reference_model_siso/include
  )

target_link_libraries(ref_model_velocity
 reference_model_siso 
)

target_link_libraries(ref_model_position
 reference_model_siso 
)
target_compile_features(ref_model_velocity PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(ref_model_position PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install ref_model_velocity files
install(TARGETS ref_model_velocity 
  DESTINATION lib/${PROJECT_NAME})

# install ref_model_velocity files
install(TARGETS ref_model_position 
  DESTINATION lib/${PROJECT_NAME})

  # install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install configuration files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
