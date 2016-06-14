# Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
# Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Authors: Pedro Vicente <pvicente@isr.ist.utl.pt>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME HandPoseEstimation-R)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_package(OPENCV)

# add include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(folder_source src/main.cpp 
		  src/pf3dtracker.cpp)
				  
set(folder_header include/pf3dTracker.hpp)
				 

include_directories(${PROJECT_SOURCE_DIR}/include)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})


# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              



# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${ICUB_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)