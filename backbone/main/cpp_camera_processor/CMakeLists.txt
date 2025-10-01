cmake_minimum_required(VERSION 3.16)
project(carla_camera_processor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Find CARLA
pkg_check_modules(CARLA REQUIRED carla)

# Find Zenoh
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZENOH REQUIRED zenoh)

# Include directories
include_directories(${CARLA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ZENOH_INCLUDE_DIRS})

# Link directories
link_directories(${CARLA_LIBRARY_DIRS})
link_directories(${ZENOH_LIBRARY_DIRS})

# Add executable
add_executable(camera_processor camera_processor.cpp)

# Link libraries
target_link_libraries(camera_processor 
    ${CARLA_LIBRARIES}
    ${OpenCV_LIBS}
    ${ZENOH_LIBRARIES}
    Threads::Threads
    rt  # For shared memory functions
)

# Compiler flags
target_compile_options(camera_processor PRIVATE
    -O3
    -march=native
    -Wall
    -Wextra
)

# Set RPATH for runtime library loading
set_target_properties(camera_processor PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "${CARLA_LIBRARY_DIRS};${ZENOH_LIBRARY_DIRS}"
)

# Install target
install(TARGETS camera_processor DESTINATION bin)

