cmake_minimum_required(VERSION 2.8.3)
project(my)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cartographer_ros_msgs
  tf2
  tf2_eigen
  tf2_ros
)

catkin_package(
	CATKIN_DEPENDS
	cartographer_ros_msgs
	tf2
    tf2_eigen
    tf2_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(amcl_sub src/amcl_sub.cpp)
add_executable(stage_sub src/stage_sub.cpp)
add_executable(laser2txt src/laser2txt.cpp)
add_executable(laser_sub_pub src/laser_sub_pub.cpp)
add_executable(landmarks_pub src/landmarks_pub.cpp)
add_executable(eigen_affine3d src/Eigen_Affine3d.cpp)
add_executable(mlandmarks_pub src/mlandmarks_pub.cpp)

target_link_libraries(mlandmarks_pub ${catkin_LIBRARIES})
target_link_libraries(eigen_affine3d ${catkin_LIBRARIES})
target_link_libraries(landmarks_pub ${catkin_LIBRARIES})
target_link_libraries(laser_sub_pub ${catkin_LIBRARIES})
target_link_libraries(laser2txt ${catkin_LIBRARIES})
target_link_libraries(amcl_sub ${catkin_LIBRARIES})
target_link_libraries(stage_sub ${catkin_LIBRARIES})

install( TARGETS
    amcl_sub stage_sub laser_sub_pub landmarks_pub eigen_affine3d mlandmarks_pub laser2txt
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
