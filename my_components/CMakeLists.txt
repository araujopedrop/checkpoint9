cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(PreApproach SHARED src/preApproach_component.cpp)
target_compile_definitions(PreApproach PRIVATE "COMPOSITION_BUILDING_DLL")

add_library(AttachServer SHARED src/attachServer_component.cpp)
target_compile_definitions(AttachServer PRIVATE "COMPOSITION_BUILDING_DLL")

add_library(AttachClient SHARED src/attachClient_component.cpp)
target_compile_definitions(AttachClient PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(PreApproach
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "custom_interfaces"
  )

ament_target_dependencies(AttachServer
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "custom_interfaces"
  )

ament_target_dependencies(AttachClient
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "custom_interfaces"
  )

rclcpp_components_register_nodes(PreApproach "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:PreApproach>\n")

rclcpp_components_register_nodes(AttachServer "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:AttachServer>\n")

rclcpp_components_register_nodes(AttachClient "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:AttachClient>\n")

install(TARGETS
  PreApproach
  AttachServer
  AttachClient
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_executable(manual_composition
  src/manual_composition.cpp)
target_link_libraries(manual_composition
  AttachServer
)
ament_target_dependencies(manual_composition
  "rclcpp")

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
