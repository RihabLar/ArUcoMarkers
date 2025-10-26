cmake_minimum_required(VERSION 3.10)
project(aruco_generator)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the OpenCV package (including the ArUco module)
find_package(OpenCV REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Specify the source files
set(SOURCES
    src/generate_marker.cpp
)

# Create the executable
add_executable(generate_marker ${SOURCES})

# Link against the OpenCV libraries
target_link_libraries(generate_marker PRIVATE ${OpenCV_LIBS})

# Add compile options (optional: optimization flags)
target_compile_options(generate_marker PRIVATE -O3 -std=c++11)

# Specify the source files for generate_board
set(GENERATE_BOARD_SOURCES src/generate_board.cpp)

# Create the generate_board executable
add_executable(generate_board ${GENERATE_BOARD_SOURCES})

# Link against the OpenCV libraries for generate_board
target_link_libraries(generate_board PRIVATE ${OpenCV_LIBS})

# Add compile options (optional: optimization flags) for generate_board
target_compile_options(generate_board PRIVATE -O3 -std=c++11)

# Specify the source files for detect_aruco
set(DETECT_ARUCO_SOURCES src/detect_aruco.cpp)

# Create the detect_aruco executable
add_executable(detect_aruco ${DETECT_ARUCO_SOURCES})

# Link against the OpenCV libraries for detect_aruco
target_link_libraries(detect_aruco PRIVATE ${OpenCV_LIBS})

# Add compile options (optional: optimization flags) for detect_aruco
target_compile_options(detect_aruco PRIVATE -O3 -std=c++11)

# Specify the source files for calibrate
set(CALIBRATE_SOURCES src/calibrate.cpp)

# Create the calibrate executable
add_executable(calibrate ${CALIBRATE_SOURCES})

# Link against the OpenCV libraries for calibrate
target_link_libraries(calibrate PRIVATE ${OpenCV_LIBS})

# Add compile options (optional: optimization flags) for calibrate
target_compile_options(calibrate PRIVATE -O3 -std=c++11)
# Specify the source files for pose_estimation
set(POSE_ESTIMATION_SOURCES src/pose_estimation.cpp)

# Create the pose_estimation executable
add_executable(pose_estimation ${POSE_ESTIMATION_SOURCES})

# Link against the OpenCV libraries for pose_estimation
target_link_libraries(pose_estimation PRIVATE ${OpenCV_LIBS})

# Add compile options (optional: optimization flags) for pose_estimation
target_compile_options(pose_estimation PRIVATE -O3 -std=c++11)


# Specify the source files for draw_cube
set(DRAW_CUBE_SOURCES src/draw_cube.cpp)

# Create the draw_cube executable
add_executable(draw_cube ${DRAW_CUBE_SOURCES})

# Link against the OpenCV libraries for draw_cube
target_link_libraries(draw_cube PRIVATE ${OpenCV_LIBS})

# Add compile options (optional: optimization flags) for draw_cube
target_compile_options(draw_cube PRIVATE -O3 -std=c++11)
