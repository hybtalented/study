echo "配置树莓派交叉编译"
###
 # @Author Youbiao He hybtalented@163.com
 # @Date 2022-10-26
 # @LastEditors Youbiao He
 # @LastEditTime 2022-10-28
 # @FilePath /raspberry/scripts/arm-env.sh
 # @Description 
### 
export RASP_WORK_DIR=/home/hybtalented/rpi-tools
export RASP_SYSROOT_DIR=${RASP_WORK_DIR}/raspberrypi/sysroot

export ARM_SYSTEM_ARCH=aarch64
export ARM_LIBRARY_ARCH=${ARM_SYSTEM_ARCH}-linux-gnu
export ARM_COMPILER_PATH=/usr