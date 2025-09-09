cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
#if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
#    add_definitions(-DRELEASE)
#endif ()

project(NE30_controller)

set(EXECUTABLE_NAME main)

set(Eigen3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/eigen-3.4.0")
set(INNFOS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/innfos/sdk/include")
set(INNFOS_LIB_DIR "${PROJECT_SOURCE_DIR}/3rdparty/innfos/sdk/lib/linux_x86_64")
set(JSON_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/nlohmann/single_include")

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/3rdparty
        ${Eigen3_INCLUDE_DIR}
        ${INNFOS_INCLUDE_DIR}
        ${JSON_INCLUDE_DIR}
)


ADD_EXECUTABLE(${EXECUTABLE_NAME} main.cpp)

if(WIN32)
        target_link_libraries(${EXECUTABLE_NAME}
                Ws2_32
    )
endif()
if(UNIX AND NOT APPLE)  
    find_package(Threads REQUIRED)
    target_link_libraries(${EXECUTABLE_NAME} Threads::Threads)

endif()
# 根据平台设置库文件扩展名和命名约定
if(WIN32)
    set(LIB_EXT ".lib")
    set(SHARED_LIB_EXT ".dll")
    set(DEBUG_SUFFIX "d")  # Windows 调试版库有 'd' 后缀
else()
    set(LIB_EXT ".so")     # 或者 ".a" 对于静态库
    set(SHARED_LIB_EXT ".so")
    set(DEBUG_SUFFIX "")   # Linux 通常没有调试后缀
endif()

# 使用生成器表达式处理不同配置
target_link_libraries(${EXECUTABLE_NAME}
    $<$<CONFIG:Release>:${INNFOS_LIB_DIR}/release/${CMAKE_STATIC_LIBRARY_PREFIX}actuatorController${DEBUG_SUFFIX}${LIB_EXT}>
    $<$<CONFIG:Debug>:${INNFOS_LIB_DIR}/debug/${CMAKE_STATIC_LIBRARY_PREFIX}actuatorController${DEBUG_SUFFIX}${LIB_EXT}>
)

# 使用生成器表达式获取可执行文件的完整路径
set(TARGET_FILE_DIR $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)

# 打印可执行文件的路径（这将在构建时发生）
add_custom_command(
    TARGET ${EXECUTABLE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Executable path: $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>"
    COMMAND ${CMAKE_COMMAND} -E echo "Build type: $<CONFIG>"
    COMMAND ${CMAKE_COMMAND} -E
    copy_if_different
    "$<$<CONFIG:Release>:${INNFOS_LIB_DIR}/release/${CMAKE_SHARED_LIBRARY_PREFIX}actuatorController${DEBUG_SUFFIX}${SHARED_LIB_EXT}>"
    "$<$<CONFIG:Debug>:${INNFOS_LIB_DIR}/debug/${CMAKE_SHARED_LIBRARY_PREFIX}actuatorController${DEBUG_SUFFIX}${SHARED_LIB_EXT}>"
    "$<TARGET_FILE_DIR:${EXECUTABLE_NAME}>"
)

# 添加调试信息
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "RELEASE mode")
    message(STATUS "Library: ${INNFOS_LIB_DIR}/release/${CMAKE_STATIC_LIBRARY_PREFIX}actuatorController${DEBUG_SUFFIX}${LIB_EXT}")
else()
    message(STATUS "DEBUG mode")
    message(STATUS "Library: ${INNFOS_LIB_DIR}/debug/${CMAKE_STATIC_LIBRARY_PREFIX}actuatorController${DEBUG_SUFFIX}${LIB_EXT}")
endif()