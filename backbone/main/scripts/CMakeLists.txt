cmake_minimum_required(VERSION 3.10)
project(cpp_zenoh_shm_processor CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Zenoh C++
find_package(zenohc REQUIRED)

# Add executable
add_executable(cpp_zenoh_shm_processor cpp_zenoh_shm_processor.cpp)

# Link libraries
target_link_libraries(cpp_zenoh_shm_processor
    PRIVATE
    ${OpenCV_LIBS}
    ${zenohc_LIBRARIES}
    rt  # for shm_open
)

# Include directories
target_include_directories(cpp_zenoh_shm_processor
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${zenohc_INCLUDE_DIRS}
    /home/dongmin/SEA-ME/Simulation/carla/LibCarla/source
)

# Compiler flags
target_compile_options(cpp_zenoh_shm_processor PRIVATE -Wall -Wextra)
