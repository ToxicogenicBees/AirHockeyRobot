cmake_minimum_required(VERSION 3.10)
project(AirHockeyLaptop)

# --- OpenCV Setup ---
set(OpenCV_DIR "C:/opencv/build-mingw/install")
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# --- Common include directories ---
set(COMMON_INCLUDES
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../shared/include
)

# --- List of executables ---
set(EXECUTABLES
    PuckPrediction
    PacketTest
    OpenCVTest
    main
)

# --- Mapping source files for each executable ---
set(PuckPrediction_SOURCES
    src/Examples/PuckPrediction.cpp
    src/Motion/MovingObject.cpp
    src/Motion/Mallet.cpp
    src/Motion/Puck.cpp
)

set(PacketTest_SOURCES
    src/Examples/PacketTest.cpp
)

set(OpenCVTest_SOURCES
    src/Examples/OpenCVTest.cpp
)

set(main_SOURCES
    src/main.cpp
    src/Motion/MovingObject.cpp
    src/Motion/Mallet.cpp
    src/Motion/Puck.cpp
)

# --- Function to create executables with common includes and OpenCV ---
function(add_opencv_executable exe_name src_list)
    add_executable(${exe_name} ${src_list})
    target_include_directories(${exe_name} PRIVATE ${COMMON_INCLUDES})
    target_link_libraries(${exe_name} PRIVATE ${OpenCV_LIBS})
endfunction()

# --- Add all executables ---
add_opencv_executable(PuckPrediction "${PuckPrediction_SOURCES}")
add_opencv_executable(PacketTest "${PacketTest_SOURCES}")
add_opencv_executable(OpenCVTest "${OpenCVTest_SOURCES}")
add_opencv_executable(main "${main_SOURCES}")