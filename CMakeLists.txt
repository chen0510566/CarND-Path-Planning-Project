project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -O3)

#set(CMAKE_BUILD_TYPE "Debug")

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/way_points.cpp src/way_points.h src/vehicle.cpp src/vehicle.h src/types.h src/utils.h src/sensor_fusion.cpp src/sensor_fusion.h src/other_vehicle.cpp src/other_vehicle.h src/trajectory.cpp src/trajectory.h src/path_planner.cpp src/path_planner.h src/vehicle_state.cpp src/vehicle_state.h src/trajectory_pool.cpp src/trajectory_pool.h src/lane_keeping.cpp src/lane_keeping.h src/lane_changing.cpp src/lane_changing.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

find_package(Glog REQUIRED)

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
include_directories(include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
