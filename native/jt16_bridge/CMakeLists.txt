cmake_minimum_required(VERSION 3.18)
project(optflow_jt16_bridge LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED HESAI_SDK_DIR)
    message(FATAL_ERROR "HESAI_SDK_DIR must point to HesaiLidar_SDK_2.0")
endif()

set(WITH_PTCS_USE OFF CACHE BOOL "" FORCE)
set(DISENABLE_TEST_CC ON CACHE BOOL "" FORCE)
add_subdirectory("${HESAI_SDK_DIR}" hesai_sdk)

add_executable(optflow-jt16-bridge jt16_bridge.cc)
target_link_libraries(optflow-jt16-bridge PRIVATE hesai_sdk_lib)
