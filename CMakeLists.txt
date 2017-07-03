project(ExtendedKF)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++14)

set(CXX_FLAGS -Wall)
set(CMAKE_CXX_FLAGS, ${CMAKE_CXX_FLAGS} ${CXX_FLAGS})

set(helpers src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp)
set(sources src/main.cpp ${helpers})
set(tests src/tests/catch_main.cpp src/tests/tools_test.cpp src/tools.cpp src/tests/fusionekf_test.cpp src/tests/kalman_filter_test.cpp)

include_directories(src)


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(ExtendedKF ${sources})
add_executable(runtest ${tests} ${helpers})

target_link_libraries(ExtendedKF z ssl uv uWS)
