cmake_minimum_required(VERSION 3.20)

project(alns)  

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

if(MSVC)
    add_compile_options(/wd5045 /wd5266 /wd4820 /wd4668 /wd4996 /wd4514 /wd4267 /wd4710 /wd4711 /wd4868)
endif()

file(GLOB SOURCES "src/*.cpp")

find_package(Threads REQUIRED)

add_executable(${CMAKE_PROJECT_NAME} main.cpp ${SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Threads::Threads)
