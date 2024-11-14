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
set(INNFOS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/innfos/include")
set(INNFOS_LIB_DIR "${PROJECT_SOURCE_DIR}/3rdparty/innfos/lib/windows_x64")

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/3rdparty
        ${Eigen3_INCLUDE_DIR}
        ${INNFOS_INCLUDE_DIR}
)


ADD_EXECUTABLE(${EXECUTABLE_NAME} main.cpp)

target_link_libraries(${EXECUTABLE_NAME}
        Ws2_32
)

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    message(STATUS "RELEASE mode")
    message(STATUS ${INNFOS_LIB_DIR}/release/actuatorController.lib)
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
            ${INNFOS_LIB_DIR}/release/actuatorController.lib
    )
else ()
    message(STATUS "DEBUG mode")
    message(STATUS ${INNFOS_LIB_DIR}/debug/actuatorControllerd.lib)
    TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME}
            ${INNFOS_LIB_DIR}/debug/actuatorControllerd.lib
    )
endif ()

# 使用生成器表达式获取可执行文件的完整路径
set(TARGET_FILE_DIR $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)

# 打印可执行文件的路径（这将在构建时发生）
add_custom_command(
        TARGET ${EXECUTABLE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Executable path: ${TARGET_FILE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E echo "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}"
        COMMAND ${CMAKE_COMMAND} -E
        copy_if_different
        "$<$<CONFIG:Release>:${INNFOS_LIB_DIR}/release/actuatorController.dll>"
        "$<$<CONFIG:Debug>:${INNFOS_LIB_DIR}/debug/actuatorControllerd.dll>"
        "${TARGET_FILE_DIR}"
)