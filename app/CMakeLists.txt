find_package(PythonLibs 2.7)
add_executable(shell-app main.cpp RRT.cpp map.cpp RRTNode.cpp)
target_include_directories(shell-app PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(shell-app ${PYTHON_LIBRARIES})

include_directories(${CMAKE_SOURCE_DIR}/include)