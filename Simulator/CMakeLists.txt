project (Client)
cmake_minimum_required(VERSION 2.6)

set (VREP_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../"
    CACHE "VRep root dir" STRING)

set (SOCCER_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Soccer"
    CACHE "Soccer root dir" STRING)

set (CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wno-reorder -pedantic -U__STRICT_ANSI__")

execute_process (COMMAND "/bin/bash" "-c" "cd ${VREP_ROOT}; echo -n `pwd`"
    OUTPUT_VARIABLE VREP_ROOT)

set (VREP_PATH
    "${VREP_ROOT}/Vrep/programming/"
    CACHE "The VRep build path" STRING)

set (CLIENT_PATH
    "${VREP_ROOT}/Client")

include_directories (${CLIENT_PATH} ${SOCCER_ROOT}/Hardware)
link_directories (${CLIENT_PATH}/build)

set (SOURCES main.cpp kinematic.cpp function.cpp)

set (SOURCES
    ${SOURCES}
    ${SOCCER_ROOT}/Hardware/HardwareServer.cpp
    ${SOCCER_ROOT}/Hardware/HardwareImage.cpp
    )

set (EXTRA_LIBRARIES pthread VREPClient)

execute_process (COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/genscript.sh"
                "${VREP_ROOT}" "${CMAKE_BINARY_DIR}/vrep")

add_executable (Client ${SOURCES} ${CLIENT_SOURCES})
target_link_libraries (Client ${EXTRA_LIBRARIES} zmq)

