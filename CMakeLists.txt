cmake_minimum_required(VERSION 3.10)
project(MEKF LANGUAGES C CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/include )


file(GLOB_RECURSE MEKF_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

# Single executable target 
add_executable(tester
    ${CMAKE_SOURCE_DIR}/testing/main.cpp
    ${MEKF_SOURCES}
)

target_include_directories(tester
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)


