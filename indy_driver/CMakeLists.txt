cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Debug)
project(indy_driver)

add_compile_options(-std=c++11 -pthread -Wl,--no-as-needed)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  industrial_robot_client
)

catkin_package(
   CATKIN_DEPENDS 
   		roscpp
		industrial_robot_client
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/SocketHandler)

add_executable(indy6d_driver 
  src/indy6d_driver.cpp 
  src/TrajectoryDownloader/JointTrajectoryDownloader.cpp
)
target_link_libraries(indy6d_driver 
  ${catkin_LIBRARIES}
  IndyDCPSocket
  industrial_robot_client
)

add_executable(indy7d_driver 
  src/indy7d_driver.cpp 
  src/TrajectoryDownloader/JointTrajectoryDownloader.cpp
)
target_link_libraries(indy7d_driver 
  ${catkin_LIBRARIES}
  IndyDCPSocket
  industrial_robot_client
)
