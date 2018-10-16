set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    MapTest.cpp
    ../app/map.cpp
    RRTNodeTest.cpp
    ../app/RRTNode.cpp
    RRTTest.cpp
    ../app/RRT.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
