 ## 
 ##    This file is part of timr.
 ## 
 ##    timr is free software: you can redistribute it and/or modify 
 ##    it under the terms of the GNU General Public License as published 
 ##    by the Free Software Foundation, either version 3 of the License, 
 ##    or (at your option) any later version.
 ## 
 ##    timr is distributed in the hope that it will be useful, 
 ##    but WITHOUT ANY WARRANTY; without even the implied warranty of 
 ##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 ##    See the GNU General Public License for more details.
 ## 
 ##    You should have received a copy of the GNU General Public License
 ##    along with timr. If not, see <https://www.gnu.org/licenses/>.
 ##

 ##
 ##    Filename CmakeLists.txt
 ##	   Author Jiawei ZHAO
 ##	   Version 1.0
 ##	   Date 2024-2025
 ##
 ##    This project includes [qpOASES](https://github.com/coin-or/qpOASES.git), 
 ##    licensed under the GNU LESSER GENERAL PUBLIC LICENSE Version 2.1, February 1999
 ##
 ##    This project includes [nlohmann/json](https://github.com/nlohmann/json), 
 ##    licensed under the MIT License 

cmake_minimum_required(VERSION 3.25)
project(timr VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_DEBUG_POSTFIX d)

# Macro options
option(timr_BUILD_EXAMPLES "Build examples." ON)
message(STATUS "timr_BUILD_EXAMPLES is set to ${timr_BUILD_EXAMPLES}")

# add compiler warning flags just when building this project via
# the BUILD_INTERFACE genex
set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
add_compile_options(
  "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
  "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
)

# Not build lib examples
set(QPOASES_BUILD_EXAMPLES OFF CACHE INTERNAL "Not build examples for qpoases.")
add_subdirectory(libs/qpOASES)
add_subdirectory(libs/json)

add_library(timr INTERFACE)
target_include_directories(timr INTERFACE 
    $<BUILD_INTERFACE:
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/libs/qpOASES/include
        ${PROJECT_SOURCE_DIR}/libs/json/include>
    $<INSTALL_INTERFACE:include> 
)
target_link_libraries(timr INTERFACE
    qpOASES
)

# Export target configuration
install(TARGETS timr
        EXPORT timrTargets
        INCLUDES DESTINATION include)

# Install the headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
    DESTINATION include 
    FILES_MATCHING PATTERN "*.hpp")

# Create and install the package configuration files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/timrConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(EXPORT timrTargets
        FILE timrTargets.cmake
        DESTINATION lib/cmake/timr)

configure_file(timrConfig.cmake.in timrConfig.cmake @ONLY)
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/timrConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/timrConfigVersion.cmake"
        DESTINATION lib/cmake/timr)


# Build examples
if(timr_BUILD_EXAMPLES)
    set(EXAMPLE_NAMES
        timr_example1
    )
    foreach(EXAMPLE ${EXAMPLE_NAMES})
        add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
        target_link_libraries(${EXAMPLE} timr)
    endforeach()
endif()