cmake_minimum_required(VERSION 3.10)
project(MEKF LANGUAGES C CXX)

# Set C/C++ compilers (optional if using defaults)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for editor tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Disable response files so compile_commands.json contains explicit -I flags
set(CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES OFF)
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES OFF)




include_directories("C:/msys64/mingw64/include/c++/15.2.0")
include_directories(${CMAKE_SOURCE_DIR}/include)

# Single executable target 
add_executable(tester
    tests/ETL.cpp
    tests/main.cpp
)

target_include_directories(tester
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        C:/msys64/mingw64/include/c++/15.2.0
    )

# target_compile_options(tester PRIVATE -isystem \"C:/msys64/mingw64/x86_64-w64-mingw32/include\")
# if(MSVC)
#     target_compile_options(tester PRIVATE /W4)
# else()
#     target_compile_options(tester PRIVATE -Wall -Wextra -pedantic -std=gnu++17)
# endif()

# Helpful: print a message about where compile_commands.json will be
# message(STATUS "compile_commands.json will be written to ${CMAKE_BINARY_DIR}/compile_commands.json")
# cmake_minimum_required(VERSION 3.10)
# project(MEKF)

# # Set C++ standard
# set(CMAKE_C_COMPILER   gcc)
# set(CMAKE_CXX_COMPILER g++)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json with expanded flags
# set(CMAKE_CXX_COMPILER_LAUNCHER "")

# # target_compile_options(tester PRIVATE -Wall -Wextra -pedantic)

# # # Add Eigen include directory
# include_directories(${CMAKE_SOURCE_DIR}/include)

# add_executable(tester)

# target_sources(tester
#     PRIVATE
#         tests/main.cpp
# )

# target_include_directories(tester
#     PUBLIC 
#         ${CMAKE_CURRENT_SOURCE_DIR}/include
#         ${CMAKE_CURRENT_SOURCE_DIR}/include/Eigen
# )
# # Optional: Add compiler flags for better error checking

# # Export compile commands and avoid response files so tools (clangd/Helix)
# # can see the full include flags inside compile_commands.json
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES OFF)
# set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES OFF)

# # Make sure includes are visible to the build and to generated compile commands
# include_directories(${CMAKE_SOURCE_DIR}/include)

# add_executable(tester tests/main.cpp)

# target_include_directories(tester
#     PUBLIC 
#         ${CMAKE_SOURCE_DIR}/include
# )

# # Optional: Add compiler flags for better error checking
# if(MSVC)
#     target_compile_options(tester PRIVATE /W4)
# else()
#     target_compile_options(tester PRIVATE -Wall -Wextra -pedantic -std=gnu++17)
# endif()

