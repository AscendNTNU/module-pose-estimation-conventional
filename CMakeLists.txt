cmake_minimum_required(VERSION 2.8.3)

##CHANGE THIS TO YOUR PACKAGE NAME
project(module-pose-estimation-conventional)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Export compile_commands.json for autocompletion
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE DEBUG)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ascend_msgs
    geometry_msgs
	vision_msgs
    roscpp
    cv_bridge
    image_transport

    tf2_ros
)

# find_package(OpenCV REQUIRED #COMPONENTS
#         #imgproc
#         #highgui
#         )

## Includes headers in include folder
catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS
    ascend_msgs
    geometry_msgs
	vision_msgs
	roscpp

    tf2_ros
)
## Includes headers in include folder
include_directories( include
    ${catkin_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
        )

# file(GLOB SOURCES "src/*.cpp")

#########################################################################################
#REPEAT THIS FOR ALL NODES, WITH UNIQUE node_name FOR EACH NODE
## Adds a ros node with name node_name
#add_executable(node_name src/node_name_node.cpp)
## Adds a compilation dependency to node_name
#add_dependencies(node_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Links node_name together with other libraries


## Pose_extraction node:
add_executable(pose_extraction_node
	src/pose_extraction_node.cpp
	src/pose_extraction.cpp
	src/blue_square_score.cpp
	src/foreground_extraction.cpp
	src/depthToWorld.cpp
        src/utils/drawFunctions.cpp
        src/utils/cvPointUtilities.cpp
		)


add_dependencies(pose_extraction_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_extraction_node
        ${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#       ${OpenCV_LIBS}
        )

add_executable(bbox_publisher_node
		src/bbox_publisher/bbox_publisher_node.cpp
		src/bbox_publisher/Bbox_publisher.cpp src/bbox_publisher/Bbox_publisher.h
		src/foreground_extraction.cpp
		)
add_dependencies(bbox_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bbox_publisher_node
		${catkin_LIBRARIES}
		#	${OpenCV_LIBRARIES}
		#       ${OpenCV_LIBS}
		)
