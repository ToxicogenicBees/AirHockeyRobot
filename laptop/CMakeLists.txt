cmake_minimum_required(VERSION 3.10)
project(AirHockeyLaptop)

# --- OpenCV Setup ---
set(OpenCV_DIR "C:/opencv/build/x64/vc16/lib")
find_package(OpenCV REQUIRED)

# --- Executables ---
add_executable(PuckPrediction
    src/Examples/PuckPrediction.cpp
    src/Motion/MovingObject.cpp
    src/Motion/Mallet.cpp
    src/Motion/Puck.cpp
)

add_executable(PacketTest
    src/Examples/PacketTest.cpp
)

add_executable(main
    src/main.cpp
    src/Motion/MovingObject.cpp
    src/Motion/Mallet.cpp
    src/Motion/Puck.cpp
)

# --- Include Directories ---
target_include_directories(PuckPrediction PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../shared/include
)

target_include_directories(PacketTest PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../shared/include
)

target_include_directories(main PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../shared/include
)

# --- Link Libraries ---
target_link_libraries(PuckPrediction PRIVATE ${OpenCV_LIBS})
target_link_libraries(PacketTest PRIVATE ${OpenCV_LIBS})
target_link_libraries(main PRIVATE ${OpenCV_LIBS})