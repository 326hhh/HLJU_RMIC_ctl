cmake_minimum_required(VERSION 3.10)
project(gimbal_ctl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(X11 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JPEG REQUIRED libjpeg)

# 复用 gimbal 工程的线协议定义 —— 只有一个源头，绝不能复制一份
set(GIMBAL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../gimbal"
    CACHE PATH "gimbal 工程根目录（用于复用 control_link.hpp）")
if(NOT EXISTS "${GIMBAL_ROOT}/include/gimbal/link/control_link.hpp")
    message(FATAL_ERROR "找不到 ${GIMBAL_ROOT}/include/gimbal/link/control_link.hpp，"
                        "请用 -DGIMBAL_ROOT=<路径> 指定 gimbal 工程位置")
endif()

add_executable(gimbal_ctl
    src/main.cpp
    src/mjpeg_view.cpp
)

target_include_directories(gimbal_ctl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GIMBAL_ROOT}/include
)
target_link_libraries(gimbal_ctl PRIVATE ${X11_LIBRARIES} ${JPEG_LIBRARIES})
target_include_directories(gimbal_ctl PRIVATE ${JPEG_INCLUDE_DIRS})
target_link_directories(gimbal_ctl PRIVATE ${JPEG_LIBRARY_DIRS})

message(STATUS "gimbal_ctl: X11 + libjpeg，协议头来自 ${GIMBAL_ROOT}")
