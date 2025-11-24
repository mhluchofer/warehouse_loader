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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(attach_shelf REQUIRED)

# include header files
include_directories(include)

# Add  Shared Library , Link Dependencies, Register components 
add_library(preapproach_component SHARED src/preapproach_component.cpp)
target_compile_definitions(preapproach_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(preapproach_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "rcl_interfaces")
rclcpp_components_register_nodes(preapproach_component "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:preapproach_component>\n")

add_library(attach_server_component SHARED src/attach_server_component.cpp)
target_compile_definitions(attach_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs"
  "attach_shelf")
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:attach_server_component>\n")

add_library(attach_client_component SHARED src/attach_client_component.cpp)
target_compile_definitions(attach_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_client_component
  "rclcpp"
  "rclcpp_components"
  "attach_shelf")
rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:attach_client_component>\n")


# Manual Composition 
add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition 
	attach_server_component)
ament_target_dependencies(manual_composition
	"rclcpp")

# Install Components 
install(TARGETS
	preapproach_component
	attach_server_component
	attach_client_component
	ARCHIVE DESTINATION lib
  	LIBRARY DESTINATION lib
  	RUNTIME DESTINATION bin)

# Install share files, launch 
install(DIRECTORY
    launch
	rviz
	DESTINATION share/${PROJECT_NAME})

# Install executbales
install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()