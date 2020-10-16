#################################################################################
# User build settings

set(DUAL_THREAD true) # Set true to process image and inertial data on different
		      # threads
set(VERBOSE true) # Set false to disable all publishing and standard output
				  # stream, except pose at update rate. That will improve runtime.
set(PROFILING false)  # Set true to disable compiler flags which are not
		      # compatible with Callgrind profiling tool.
set(UNIT_TESTS false) # Set true to enable unit tests

################################################################################ 

cmake_minimum_required(VERSION 2.8.3)
project(x_vio_ros)
set (CMAKE_BUILD_TYPE Release)

# Set definitions
if(DUAL_THREAD)
  add_definitions(-DDUAL_THREAD)
endif()
if(VERBOSE)
  add_definitions(-DVERBOSE)
endif()
if(UNIT_TESTS)
  add_definitions(-DRUN_UNIT_TESTS)
endif()
if (CMAKE_BUILD_TYPE MATCHES Debug)
  add_definitions(-DDEBUG -DDEBUGMSF)
elseif (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  # Enable asserts
  add_definitions(-UNDEBUG)
endif()
add_definitions(-D_LINUX -D_REENTRANT)

# OpenCV: catkin package or system install
# first try locate the opencv3_catkin ros package
# else try OpenCV 3 and 4 system installs.
find_package(opencv3_catkin QUIET)
find_package(OpenCV 3 QUIET)
find_package(OpenCV 4 QUIET)

if(opencv3_catkin_FOUND)
  message("OpenCV3_catkin detected")
  set(OPENCV_PACKAGE "opencv3_catkin")
  set(OpenCV_INCLUDE_DIRS "")
  set(OpenCV_LIBRARIES "")
elseif(${OpenCV_FOUND})
  message("OpenCV system install detected")
  set(OPENCV_PACKAGE "")
else()
  message(FATAL_ERROR "No OpenCV 3 or 4 detected.")
endif()

find_package(catkin REQUIRED COMPONENTS
  x 
  ${OPENCV_PACKAGE}
  cv_bridge
  image_geometry
  dynamic_reconfigure
  image_transport
  roscpp
  tf
  message_generation
  sensor_msgs
  std_msgs
)

# Set build flags, depending on the architecture
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

if (CMAKE_BUILD_TYPE MATCHES Debug)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
endif()

if (CMAKE_BUILD_TYPE MATCHES Release)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64") # tested on Jetson TX2
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+crypto -mcpu=cortex-a57+crypto -flto -ffast-math -fvect-cost-model=unlimited")
  #elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch32") # uncomment with correct check for Snapdragon Flight Pro
  #  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=softfp -flto -ffast-math -fvect-cost-model=unlimited")
  endif()

  if (${PROFILING} MATCHES false)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros")
  endif()
endif()

# Configure this package
generate_dynamic_reconfigure_options(
  cfg/xvio.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories (
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# ROS node executable
#####################
set (SOURCE
  src/x_vio_ros/node.cpp
  src/x_vio_ros/main.cpp
)

add_executable (x_vio_ros ${SOURCE})

# Make sure configure headers are built before any node using them
add_dependencies(x_vio_ros ${PROJECT_NAME}_gencfg)

# Additional libraries to link against
target_link_libraries(x_vio_ros
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Testing
#########
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(utest_node test/test_vio.test test/utest.cpp)
  target_link_libraries(utest_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${rostest_LIBRARIES})
endif()
