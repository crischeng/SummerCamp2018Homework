project(cmake_learning)
cmake_minimum_required(VERSION 3.5)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
set(EXE_SRC  ../src/AppDemo/main.cpp)
set(STATIC_SRC ../src/StaticLibDemo/StaticLibDemo.cpp)
set(SHARED_SRC ../src/SharedLibDemo/SharedLibDemo.cpp)
set(FIND_SRC ../src/AppDemo/test_for_findpackage.cpp)

find_package(Hello REQUIRED)

include_directories( ../StaticLibDemo ../SharedLibDemo)
include_directories(${Hello_INCLUDE_DIR})

add_library(SharedLibDemo SHARED ${SHARED_SRC})
add_library(StaticLibDemo ${STATIC_SRC})

add_executable(AppDemo ${EXE_SRC})
target_link_libraries(AppDemo SharedLibDemo StaticLibDemo)



add_executable(HelloDemo ${FIND_SRC})
target_link_libraries(HelloDemo ${Hello_LIBRARIES})



install(
    TARGETS AppDemo HelloDemo  SharedLibDemo StaticLibDemo
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
)



