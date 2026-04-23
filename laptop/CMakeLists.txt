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
    main
)

# --- Mapping source files for each executable ---
set(main_SOURCES
    src/main.cpp
    src/Motion/MovingObject.cpp
    src/Motion/Table.cpp
    src/Tracking/PuckTracker.cpp
    src/Tracking/CameraTracker.cpp
    src/Tracking/PhysicsTracker.cpp
    src/Tracking/NoOperationTracker.cpp
    src/Tracking/TrackerOverlay.cpp
    src/Routines/Routine.cpp
    src/Routines/BasicDefenseRoutine.cpp
    src/Routines/MotionTestRoutine.cpp
    src/Routines/StrikeTestRoutine.cpp
    src/Routines/StrikingRoutine.cpp
    src/Routines/DodgeRoutine.cpp
    src/Routines/AdvancedDefenseRoutine.cpp
    src/Routines/BasicOffenseRoutine.cpp
    src/Routines/ManualRoutine.cpp
    src/Routines/GoalDefenseRoutine.cpp
    src/Routines/NoOperationRoutine.cpp
)

# --- Function to create executables with common includes and OpenCV ---
function(add_opencv_executable exe_name src_list)
    add_executable(${exe_name} ${src_list})
    target_include_directories(${exe_name} PRIVATE ${COMMON_INCLUDES})
    target_link_libraries(${exe_name} PRIVATE ${OpenCV_LIBS})
endfunction()

# --- Add all executables ---
add_opencv_executable(main "${main_SOURCES}")