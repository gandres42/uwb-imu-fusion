cmake_minimum_required(VERSION 3.25)

project (uwb-imu-fusion)


set(SOURCES
    src/main.cpp
)

add_executable(uwb-imu-fusion ${SOURCES})

add_subdirectory(libs/navx)
target_link_libraries(uwb-imu-fusion PUBLIC navx)

target_include_directories(uwb-imu-fusion
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/libs/navx/include
)

