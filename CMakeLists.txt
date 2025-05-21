cmake_minimum_required(VERSION 3.10)
project(reference_model_ros2)
# project(reference_model_ros2 VERSION 1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)

# ROS dependencies
set(ROS_DEPS
    ament_cmake
    rclcpp
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
)

# find dependencies
foreach(dep IN LISTS ROS_DEPS)
    find_package(${dep} REQUIRED)
endforeach()

find_package(reference_model_siso REQUIRED)

# executables
add_executable(ref_model_velocity bin/main_velocity.cpp src/ref_model_velocity.cpp)
add_executable(ref_model_position bin/main_position.cpp src/ref_model_position.cpp)

foreach(target ref_model_velocity ref_model_position)
    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${target} reference_model_siso::reference_model_siso)
    ament_target_dependencies(${target} ${ROS_DEPS}) #also invokes find_package
    target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)
endforeach()

# install package files
foreach(target ref_model_velocity ref_model_position)
    install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

# install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install config file
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
