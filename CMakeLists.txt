cmake_minimum_required(VERSION 3.10)
project(CircuitDemo VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Find Threads
find_package(Threads REQUIRED)

# Enable Qt automoc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Create executable
add_executable(circuit-demo
    main.cpp
    mainwindow.cpp
    mainwindow.h
    netlistparser.h
    circuitgraph.h
    src/elk/elk_circuit_layout.cpp
    src/elk/elk_graph.cpp
)

# Test executable for 1_2_yosys.v
add_executable(test_1_2_yosys
    test_1_2_yosys.cpp
    netlistparser.h
    circuitgraph.h
    src/elk/elk_circuit_layout.cpp
    src/elk/elk_graph.cpp
)

# Metrics test executable
add_executable(test_metrics
    test_metrics.cpp
    netlistparser.h
    circuitgraph.h
    src/elk/elk_circuit_layout.cpp
    src/elk/elk_graph.cpp
)

# Simple check test executable
add_executable(test_simple_check
    test_simple_check.cpp
    netlistparser.h
    circuitgraph.h
    src/elk/elk_circuit_layout.cpp
    src/elk/elk_graph.cpp
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
    Threads::Threads
)

target_link_libraries(test_1_2_yosys
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Threads::Threads
)

target_link_libraries(test_metrics
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Threads::Threads
)

target_link_libraries(test_simple_check
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Threads::Threads
)

# elkjs metrics test executable
add_executable(test_metrics_elkjs
    test_metrics_elkjs.cpp
    netlistparser.h
    circuitgraph.h
    src/elk/elk_circuit_layout.cpp
    src/elk/elk_graph.cpp
)

target_link_libraries(test_metrics_elkjs
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Threads::Threads
)

# Install target
install(TARGETS circuit-demo DESTINATION bin)
