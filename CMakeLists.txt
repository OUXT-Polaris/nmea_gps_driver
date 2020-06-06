cmake_minimum_required(VERSION 3.5)
project(nmea_gps_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(nmea_gps_driver_component SHARED
  src/nmea_gps_driver_component.cpp)
target_compile_definitions(nmea_gps_driver_component
  PRIVATE "NMEA_GPS_DRIVER_NMEA_GPS_DRIVER_COMPONENT_BUILDING_DLL")
target_link_libraries(nmea_gps_driver_component ${Boost_LIBRARIES})
ament_target_dependencies(nmea_gps_driver_component
  rclcpp
  rclcpp_components
  nmea_msgs)
rclcpp_components_register_nodes(nmea_gps_driver_component
  "nmea_gps_driver::NmeaGpsDriverComponent")

add_executable(nmea_gps_driver_node
  src/nmea_gps_driver_node.cpp)
ament_target_dependencies(nmea_gps_driver_node
  rclcpp
  rclcpp_components
  nmea_msgs)
target_link_libraries(nmea_gps_driver_node nmea_gps_driver_component)

install(TARGETS
  nmea_gps_driver_node
  DESTINATION lib/nmea_gps_driver
)
install(TARGETS
  nmea_gps_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(nmea_gps_driver_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
