#
# Find the includes and libraries files for RKMODS such as rga mpp jsoncpp ...
# This module defines
# RKMODS_INCLUDE_DIR, where to find header files of moudules...
# RKMODS_LIBRARIES, the libraries to link against to use moudules.
# RKMODS_FOUND, If false, do not try to use complie this project.
FIND_PATH(OPENCV_HEAD_DIR opencv2
    PATHS
    /usr/local/include
    /usr/include
    PATH_SUFFIXES
    opencv4)

IF(NOT OPENCV_HEAD_DIR)
    message(FATAL_ERROR "opencv2 headers not found!")
ENDIF()

FIND_PATH(RGA_HEAD_DIR rga
    PATHS
    /usr/local/include
    /usr/include)

IF(NOT RGA_HEAD_DIR)
    message(FATAL_ERROR "rga headers not found!")
ENDIF()

FIND_PATH(JSONCPP_HEAD_DIR json.h
    PATHS
    ${SYSROOT_PATH}/include
    /usr/local/include
    /usr/include
    PATH_SUFFIXES
    json)

IF(NOT JSONCPP_HEAD_DIR)
    message(FATAL_ERROR "jsoncpp headers not found!")
ENDIF()

FIND_PATH(ROCKCHIP_HEAD_DIR rockchip
    PATHS
    /usr/local/include
    /usr/include)

IF(NOT ROCKCHIP_HEAD_DIR)
    message(FATAL_ERROR "rockchip headers not found!")
ENDIF()

FIND_PATH(LIBDRM_HEAD_DIR libdrm
    PATHS
    /usr/local/include
    /usr/include)

IF(NOT LIBDRM_HEAD_DIR)
    message(FATAL_ERROR "libdrm headers not found!")
ENDIF()

set(RKAIQ_SYSROOT "/home/wyh/SDK_DIR/rk-linux5.10-SDK-20260122/buildroot/output/rockchip_rk3588/host/aarch64-buildroot-linux-gnu/sysroot")
 
FIND_PATH(RKAIQ_HEAD_DIR rkaiq
    PATHS
    ${RKAIQ_SYSROOT}/usr/include
    /usr/local/include
    /usr/include)
 
IF(NOT RKAIQ_HEAD_DIR)
    message(FATAL_ERROR "rkaiq headers not found!")
ENDIF()

SET(RKMODS_INCLUDES
    ${RGA_HEAD_DIR}/rga
    ${JSONCPP_HEAD_DIR}/jsoncpp
    ${ROCKCHIP_HEAD_DIR}/rockchip
    ${LIBDRM_HEAD_DIR}/libdrm
    ${RKAIQ_HEAD_DIR}/rkaiq
    ${RKAIQ_HEAD_DIR}/rkaiq/xcore
    ${RKAIQ_HEAD_DIR}/rkaiq/algos
    ${RKAIQ_HEAD_DIR}/rkaiq/common
    ${RKAIQ_HEAD_DIR}/rkaiq/ipc_server
    ${RKAIQ_HEAD_DIR}/rkaiq/iq_parser
    ${RKAIQ_HEAD_DIR}/rkaiq/iq_parser_v2
    ${RKAIQ_HEAD_DIR}/rkaiq/smartIr
    ${RKAIQ_HEAD_DIR}/rkaiq/uAPI2)

FIND_LIBRARY(RGA_LIBRARY rga
    /usr/local/lib
    /usr/lib
)

IF(NOT RGA_LIBRARY)
    message(FATAL_ERROR "librga not found!")
ENDIF()

FIND_LIBRARY(JSONCPP_LIBRARY jsoncpp
    /usr/local/lib
    /usr/lib
)

IF(NOT JSONCPP_LIBRARY)
    message(FATAL_ERROR "libjsoncpp not found!")
ENDIF()

FIND_LIBRARY(RK_MPP_LIBRARY rockchip_mpp
    /usr/local/lib
    /usr/lib
)

IF(NOT RK_MPP_LIBRARY)
    message(FATAL_ERROR "librk-mpp not found!")
ENDIF()

FIND_LIBRARY(V4L2_MPP_LIBRARY v4l-rkmpp
    /usr/local/lib
    /usr/lib
    PATH_SUFFIXES
    libv4l/plugins
)

IF(NOT V4L2_MPP_LIBRARY)
    message(FATAL_ERROR "libv4l-rkmpp not found!")
ENDIF()

FIND_LIBRARY(DRM_LIBRARY drm
    /usr/local/lib
    /usr/lib
)

IF(NOT DRM_LIBRARY)
    message(FATAL_ERROR "libdrm not found!")
ENDIF()

FIND_LIBRARY(GUIDE_LIBRARY libGuideUSB2LiveStream.a
    /usr/local/lib
    /usr/lib
)

IF(NOT GUIDE_LIBRARY)
    message(FATAL_ERROR "libGuideUSB2LiveStream not found!")
ENDIF()

FIND_LIBRARY(RKAIQ_LIBRARY rkaiq
    ${RKAIQ_SYSROOT}/usr/lib
    /usr/local/lib
    /usr/lib
)
 
IF(NOT RKAIQ_LIBRARY)
    message(FATAL_ERROR "librkaiq not found!")
ENDIF()

SET(RKMODS_LIBRARIES
    ${RGA_LIBRARY}
    ${JSONCPP_LIBRARY} 
    ${RK_MPP_LIBRARY}
    ${V4L2_MPP_LIBRARY}
    ${DRM_LIBRARY}
    ${GUIDE_LIBRARY}
    ${RKAIQ_LIBRARY}) 

SET(RKMODS_FOUND "YES")  