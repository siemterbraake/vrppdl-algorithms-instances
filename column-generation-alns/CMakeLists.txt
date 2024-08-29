cmake_minimum_required(VERSION 3.20)

project(column_generation)

set(CMAKE_CXX_STANDARD 20)
option(CXX "enable C++ compilation" ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

if(MSVC)
    add_compile_options(/wd5045 /wd5266 /wd4820 /wd4668 /wd4996 /wd4514 /wd4267 /wd4710 /wd4711 /wd4868 /wd4365 /wd4242 /wd4244)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

file(GLOB SOURCES "src/*.cpp")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(${CMAKE_PROJECT_NAME} optimized ${GUROBI_CXX_LIBRARY}
                        debug ${GUROBI_CXX_DEBUG_LIBRARY})

target_link_libraries(${CMAKE_PROJECT_NAME} ${GUROBI_LIBRARY})

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
