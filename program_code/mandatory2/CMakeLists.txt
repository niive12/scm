project(mandatory2)
cmake_minimum_required(VERSION 2.8)
IF( NOT DEFINED CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
# disallow in-source builds
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
aux_source_directory(. SRC_LIST)

#Qt Creator Projects view header file inclusion.
#Not harmful nor necessary if you don't use Qt creator.
#add_executable(${PROJECT_NAME} ${SRC_LIST}) #would also be fine.
#In that case, you could use the File System view instead of Projects view.
file(GLOB SRCDIR_HPPS RELATIVE ${CMAKE_SOURCE_DIR} "*.hpp")
file(GLOB SRCDIR_HS RELATIVE ${CMAKE_SOURCE_DIR} "*.h")
add_executable(${PROJECT_NAME} ${SRC_LIST} ${SRCDIR_HPPS} ${SRCDIR_HS})
# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE true)


# ----------------- User settings ----------------- #

add_definitions("-std=c++11") # Enable C++11.
# add_definitions("-std=c++1y") # Enable C++14
#set(CMAKE_CXX_FLAGS "-Wall -O3 -v")

#Really strict and detailed compiler warnings:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Woverloaded-virtual -Weffc++ -O3 -v")

# ------------------------------------------------- #


# The name of the project
MESSAGE(${PROJECT_NAME} ":")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})
# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM ${ROBWORK_INCLUDE_DIRS} ) #SYSTEM flag is for ignoring RobWork compiler warnings.
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# The shared library to build:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ROBWORK_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")

