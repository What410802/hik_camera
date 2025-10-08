cmake_minimum_required(VERSION 3.8)

# set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
project(hik_camera)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_compile_options($<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-Ofast>)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Special processing of MVS includes and libs
set(MVS "MVS")
if(NOT EXISTS MVS) # If not included in the project
	set(MVS "/opt/MVS")
endif()
include_directories(${MVS}/include)
link_directories(${MVS}/lib/64)
# target_include_directories(hik_camera PUBLIC
#   ${MVS}/include
# )

# Auto add
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED
	src/hik_camera_node.cpp
)
# Manual add
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(camera_info_manager REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)

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

# add_executable(${PROJECT_NAME} src/hik_camera_node.cpp)

ament_target_dependencies(${PROJECT_NAME}
	"rclcpp"
	"sensor_msgs"
	# "cv_bridge"
	# "image_transport"
	"rclcpp_components"
	# "OpenCV"
)
target_link_libraries(${PROJECT_NAME}
	FormatConversion
	MediaProcess
	MvCameraControl
	MVRender
	MvUsb3vTL
)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)  # Require C++17

# 安装节点
install(TARGETS ${PROJECT_NAME}
	DESTINATION lib/${PROJECT_NAME}
)

# 安装Launch文件
install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN camera_node::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
