cmake_minimum_required(VERSION 3.5)
project(dji_sdk_demo_linux C CXX)

set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")
# set(CMAKE_C_COMPILER "gcc")
# set(CMAKE_CXX_COMPILER "g++")
add_definitions(-D_GNU_SOURCE)

if (NOT USE_SYSTEM_ARCH)
    add_definitions(-DSYSTEM_ARCH_LINUX)
endif ()

if (BUILD_TEST_CASES_ON MATCHES TRUE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
endif ()

set(PACKAGE_NAME payloadsdk)

## "uname -m" to auto distinguish Manifold2-G or Manifold2-C
# execute_process(COMMAND uname -m
#         OUTPUT_VARIABLE DEVICE_SYSTEM_ID)

SET(DEVICE_SYSTEM_ID aarch64)
if (DEVICE_SYSTEM_ID MATCHES x86_64)
    set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
    set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
    add_definitions(-DPLATFORM_ARCH_aarch64=1)
else ()
    message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif ()

file(GLOB_RECURSE MODULE_COMMON_SRC ../common/*.c)
file(GLOB_RECURSE MODULE_HAL_SRC hal/*.c)
file(GLOB_RECURSE MODULE_APP_SRC application/*.c)
file(GLOB_RECURSE MODULE_SAMPLE_SRC ../../../module_sample/*.c)
file(GLOB_RECURSE CAMERA ../../../module_sample/camera/*.c*)

include_directories(../../../module_sample)
include_directories(../common)
include_directories(application)
include_directories(../../../../../psdk_lib/include)
link_directories(../../../../../psdk_lib/lib/${TOOLCHAIN_NAME})
link_libraries(${CMAKE_CURRENT_LIST_DIR}/../../../../../psdk_lib/lib/${TOOLCHAIN_NAME}/lib${PACKAGE_NAME}.a)

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif ()

add_executable(${PROJECT_NAME}
        ${MODULE_APP_SRC}
        ${MODULE_SAMPLE_SRC}
        ${MODULE_COMMON_SRC}
        ${MODULE_HAL_SRC}
        ${CAMERA}
        )

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../common/3rdparty)

find_package(RKMODS REQUIRED)
if(RKMODS_FOUND)
    message(STATUS "Found  necessary modules installed in the system")
    message(STATUS " - Includes: ${RKMODS_INCLUDES}")
    message(STATUS " - Libraries: ${RKMODS_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${RKMODS_INCLUDES})
    target_link_libraries(${PROJECT_NAME} ${RKMODS_LIBRARIES})
else()
    message(STATUS "Cannot Find RKMODS")
endif()

find_package(OPUS REQUIRED)
if (OPUS_FOUND)
    message(STATUS "Found OPUS installed in the system")
    message(STATUS " - Includes: ${OPUS_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${OPUS_LIBRARY}")

    add_definitions(-DOPUS_INSTALLED)
    include_directories(${OPUS_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} opus)
else ()
    message(STATUS "Cannot Find OPUS")
endif (OPUS_FOUND)

find_package(LIBUSB REQUIRED)
if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
    include_directories(${LIBUSB_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} usb-1.0)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)

find_package(PNG REQUIRED)
if (PNG_FOUND)
    message(STATUS "Found PNG installed in the system")
    message(STATUS " - Includes: ${PNG_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${PNG_LIBRARIES}")

    include_directories(${PNG_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${PNG_LIBRARIES})
else ()
    message(STATUS "Cannot Find PNG")
endif (PNG_FOUND)

# FreeType for OSD text rendering
find_package(Freetype REQUIRED)
if (FREETYPE_FOUND)
    message(STATUS "Found Freetype installed in the system")
    message(STATUS " - Includes: ${FREETYPE_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${FREETYPE_LIBRARIES}")
    include_directories(${FREETYPE_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES})
else ()
    message(FATAL_ERROR "Cannot Find Freetype")
endif ()

# 增加jsoncpp支持
FIND_LIBRARY(JSONCPP_LIBRARY jsoncpp
    /usr/local/lib
    /usr/lib
)
IF(NOT JSONCPP_LIBRARY)
    message(FATAL_ERROR "libjsoncpp not found!")
ENDIF()

FIND_PATH(JSONCPP_HEAD_DIR jsoncpp
    PATHS
    /usr/local/include
    /usr/include)
IF(NOT JSONCPP_HEAD_DIR)
    message(FATAL_ERROR "jsoncpp headers not found!")
ENDIF()

target_include_directories(${PROJECT_NAME} PUBLIC ${JSONCPP_HEAD_DIR})
target_link_libraries(${PROJECT_NAME} ${JSONCPP_LIBRARY})

# add simple bgc api
target_link_libraries(${PROJECT_NAME} m dl $<TARGET_OBJECTS:SBGC_API>)

add_custom_command(TARGET ${PROJECT_NAME}
        PRE_LINK COMMAND cmake ..
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
