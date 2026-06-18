##################################
# Configure rk3588 cross-compilation
#################################
set(CMAKE_SYSTEM_NAME Linux)     #设置目标系统名字
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_FIND_ROOT_PATH    /home/wyh/SDK_DIR/rk-linux5.10-SDK-20260122/buildroot/board/rockchip/rk3588/fs-overlay
                            /home/wyh/SDK_DIR/rk-linux5.10-SDK-20260122/buildroot/output/rockchip_rk3588/host/aarch64-buildroot-linux-gnu/sysroot)
                            # /home/lee/rk3588-linux_2023/buildroot/output/rockchip_rk3588/host/aarch64-buildroot-linux-gnu/sysroot
                            # /home/lee/rk3588-linux_2023/buildroot/board/rockchip/rk3588/fs-overlay/)
                            # /home/lee/rk3588-linux_2023/buildroot/output/rockchip_rk3588/target)
                            #/home/wyh/SDK_DIR/rk-linux5.10-SDK-20260122/buildroot/output/rockchip_rk3588/target

set(CMAKE_C_COMPILER /home/wyh/SDK_DIR/rk-linux5.10-SDK-20260122/buildroot/output/rockchip_rk3588/host/bin/aarch64-buildroot-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /home/wyh/SDK_DIR/rk-linux5.10-SDK-20260122/buildroot/output/rockchip_rk3588/host/bin/aarch64-buildroot-linux-gnu-g++)

# Adjust the default behavior of the FIND_XXX() commands:
# Search programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Search headers and libraries in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
#################################
# end
##################################
