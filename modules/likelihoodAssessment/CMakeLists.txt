# Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
# Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Authors: Pedro Vicente <pvicente@isr.ist.utl.pt>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME CUDAcompareEdge2)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(CUDA)
FIND_package(OPENCV)


# add include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

# Not shure if this is the better way to include this
include_directories(${CUDA_SDK_ROOT_DIR}/inc)

set(folder_source src/likelihood.cpp
                  src/Cuda_Gl.cu)
				  
#set(folder_header include/include.hpp)
			 

include_directories(${PROJECT_SOURCE_DIR}/include)



source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})


SET (LIB_TYPE SHARED)

cuda_add_library(${PROJECTNAME} ${LIB_TYPE} ${folder_source} ${folder_header})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              


TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_SDK_ROOT_DIR}/lib/x64/freeglut.lib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
