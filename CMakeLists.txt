 ## 
 ##    This file is part of dqbot.
 ## 
 ##    dqbot is free software: you can redistribute it and/or modify 
 ##    it under the terms of the GNU General Public License as published 
 ##    by the Free Software Foundation, either version 3 of the License, 
 ##    or (at your option) any later version.
 ## 
 ##    dqbot is distributed in the hope that it will be useful, 
 ##    but WITHOUT ANY WARRANTY; without even the implied warranty of 
 ##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 ##    See the GNU General Public License for more details.
 ## 
 ##    You should have received a copy of the GNU General Public License
 ##    along with dqbot. If not, see <https://www.gnu.org/licenses/>.
 ##

 ##
 ##    Filename CmakeLists.txt
 ##	   Author Jiawei ZHAO
 ##	   Version 1.0
 ##	   Date 2024-2025
 ##
 ##    This project includes [qpOASES](https://github.com/coin-or/qpOASES.git), 
 ##    licensed under the GNU LESSER GENERAL PUBLIC LICENSE Version 2.1, February 1999

cmake_minimum_required(VERSION 3.25)

# Not build lib examples
set(QPOASES_BUILD_EXAMPLES OFF CACHE INTERNAL "Not build examples for qpoases.")

# Macro options
Option(jwbot_BUILD_EXAMPLES ON "Build examples for dqbot")
message(STATUS "jwbot_BUILD_EXAMPLES is set to ${jwbot_BUILD_EXAMPLES}")

# Set the project name and version
project(dqbot VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_DEBUG_POSTFIX d)

# add compiler warning flags just when building this project via
# the BUILD_INTERFACE genex
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
add_compile_options(
  "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
  "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)

# Make static, shared, executables all built in build dir
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

add_subdirectory(libs/qpOASES)
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/libs/qpOASES/include
)
link_libraries(
    qpOASES
)

if(jwbot_BUILD_EXAMPLES)
    set(EXAMPLE_NAMES
        dqexample1
    )
    foreach(EXAMPLE ${EXAMPLE_NAMES})
        add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
    endforeach()
endif()

# Install the headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp")
