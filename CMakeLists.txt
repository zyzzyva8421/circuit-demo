cmake_minimum_required(VERSION 3.10)
project(CircuitDemo VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Find Threads
# find_package(Threads REQUIRED) - Not needed for QProcess usually, but good to keep

# Enable Qt automoc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/include
)

# Create executable
add_executable(circuit-demo
    main.cpp
    mainwindow.cpp
    mainwindow.h
    netlistparser.h
    circuitgraph.h
)

# Copy layout_script.js to build directory
add_custom_command(TARGET circuit-demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/layout_script.js
        $<TARGET_FILE_DIR:circuit-demo>/layout_script.js
)

# Link libraries
target_link_libraries(circuit-demo
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

# Install target
install(TARGETS circuit-demo DESTINATION bin)
