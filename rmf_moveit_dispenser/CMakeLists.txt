cmake_minimum_required(VERSION 3.8)
project(rmf_moveit_dispenser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
list(APPEND CMAKE_PREFIX_PATH "/usr/include/x86_64-linux-gnu/curl")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(CURL REQUIRED)
add_executable(moveit_dispenser src/moveit_dispenser.cpp)
add_executable(moveit_ingestor src/moveit_ingestor.cpp)
add_executable(moveit_ingestor_dummy src/moveit_ingestor_dummy.cpp)
# add_library(moveit_dispenser SHARED src/moveit_dispenser.cpp)
add_library(json_header STATIC "${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann/json.hpp")
set_target_properties(json_header PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(json_header PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann")


target_link_libraries(moveit_dispenser
    CURL::libcurl
    json_header
)

ament_target_dependencies(moveit_dispenser
    rclcpp
    geometry_msgs
    rmf_dispenser_msgs
    rmf_ingestor_msgs
    moveit_ros_planning_interface
)

target_link_libraries(moveit_ingestor
    CURL::libcurl
    json_header
)

ament_target_dependencies(moveit_ingestor
    rclcpp
    geometry_msgs
    rmf_dispenser_msgs
    rmf_ingestor_msgs
    moveit_ros_planning_interface
)
target_link_libraries(moveit_ingestor_dummy
    CURL::libcurl
    json_header
)

ament_target_dependencies(moveit_ingestor_dummy
    rclcpp
    geometry_msgs
    rmf_dispenser_msgs
    rmf_ingestor_msgs
    moveit_ros_planning_interface
)
install(TARGETS
    moveit_dispenser
    moveit_ingestor
    moveit_ingestor_dummy
    DESTINATION lib/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_package()
