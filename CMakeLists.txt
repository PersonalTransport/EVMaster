cmake_minimum_required(VERSION 3.2)
project(EvMaster)

include(../cmake-xc16/MPLABX.cmake)

set(SOURCES
    src/main.c

    gen/ev_master.c
)

set(HEADERS
    gen/ev_master.h
)

set(NODE_FILES
    ev_master.ncf
    ../LightingNode/lighting.ncf
    ../SpeedometerNode/speedometer.ncf
    ../EnergyManagementNode/energy_management.ncf
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/gen/ev_master.h ${CMAKE_CURRENT_SOURCE_DIR}/gen/ev_master.c
                   COMMAND LIN -t PIC${CMAKE_SYSTEM_PROCESSOR} -i uart1 ${NODE_FILES}
                   DEPENDS ${NODE_FILES}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(EvMaster ${SOURCES} ${HEADERS} ${NODE_FILES})
target_compile_definitions(EvMaster PUBLIC -DFCY=6000000)
target_compile_options(EvMaster PUBLIC -std=gnu99)
target_include_directories(EvMaster PUBLIC include gen)
generate_mplabx_project(EvMaster)
