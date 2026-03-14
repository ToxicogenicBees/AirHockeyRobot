cmake_minimum_required(VERSION 3.10)
project(AirHockeyLaptop)

if(WIN32)
    add_definitions(-DWIN32_LEAN_AND_MEAN)
endif()

# --- OpenCV Setup ---
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
    PhysicsSim
    PacketTest
    OpenCVTest
    GraphTest
    main
)

# --- Mapping source files for each executable ---
set(PacketTest_SOURCES
    src/Examples/PacketTest.cpp
)

set(PhysicsSim_SOURCES
    src/Examples/PhysicsSim.cpp
    src/Motion/MovingObject.cpp
    src/Simulation/Table.cpp
    src/Motion/Mallet.cpp
    src/Motion/Puck.cpp
    src/Motion/PuckTracker.cpp
    src/Motion/Routines/Routine.cpp
    src/Motion/Routines/BasicDefenseRoutine.cpp
    src/Motion/Routines/MotionTestRoutine.cpp
)

set(OpenCVTest_SOURCES
    src/Examples/OpenCVTest.cpp
)

set(GraphTest_SOURCES
    src/Examples/GraphTest.cpp
    src/Visuals/Graph.cpp
)

set(main_SOURCES
    src/main.cpp
    src/State/StateTracker.cpp
    src/State/KeyLog.cpp
    src/Motion/MovingObject.cpp
    src/Simulation/Table.cpp
    src/Motion/Mallet.cpp
    src/Motion/Puck.cpp
    src/Motion/PuckTracker.cpp
    src/Motion/Routines/Routine.cpp
    src/Motion/Routines/BasicDefenseRoutine.cpp
    src/Motion/Routines/MotionTestRoutine.cpp
)

# --- Function to create executables with common includes and OpenCV ---
function(add_opencv_executable exe_name src_list)
    add_executable(${exe_name} ${src_list})
    target_include_directories(${exe_name} PRIVATE ${COMMON_INCLUDES})
    target_link_libraries(${exe_name} PRIVATE ${OpenCV_LIBS})
endfunction()

# --- Add all executables ---
add_opencv_executable(PhysicsSim "${PhysicsSim_SOURCES}")
add_opencv_executable(PacketTest "${PacketTest_SOURCES}")
add_opencv_executable(OpenCVTest "${OpenCVTest_SOURCES}")
add_opencv_executable(GraphTest "${GraphTest_SOURCES}")
add_opencv_executable(main "${main_SOURCES}")