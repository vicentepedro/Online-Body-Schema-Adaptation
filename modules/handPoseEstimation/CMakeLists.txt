# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Authors: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.9)

set(PROJECTNAME handPoseEstimation)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUBcontrib)
FIND_package(OpenCV)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpIDL)
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)
source_group("IDL Files" FILES ${PROJECT_NAME}.thrift)
source_group("DOC Files" FILES ${PROJECT_NAME}.xml)

##MINE
set(IDL_GENERATED_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(${PROJECTNAME}.thrift ${IDL_GENERATED_CODE_DIR})

include_directories(${YARP_INCLUDE_DIRS})
add_definitions(${YARP_DEFINES} -D_USE_MATH_DEFINES)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${IDL_GENERATED_CODE_DIR}/include)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(${PROJECT_NAME} ${source} ${header} ${PROJECT_NAME}.thrift ${PROJECT_NAME}.xml)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES}) 

install(TARGETS ${PROJECTNAME} DESTINATION bin)

icubcontrib_add_uninstall_target()

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${PROJECT_NAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

# required by Windows?
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")              
