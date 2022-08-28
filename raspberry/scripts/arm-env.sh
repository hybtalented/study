echo "配置树莓派交叉编译"
###
 # @Author Youbiao He hybtalented@163.com
 # @Date 2022-08-27
 # @LastEditors Youbiao He
 # @LastEditTime 2022-08-27
 # @FilePath /raspberry/scripts/arm-env.sh
 # @Description 
### 
export RASP_WORK_DIR=/home/$USER/rpi-tools
export RASP_SYSROOT_DIR=${RASP_WORK_DIR}/raspberrypi/sysroot

# export ARM_TOOL_DIR=${RASP_WORK_DIR}/tools
# export ARM_COMPILER_VERSION=8.3-raspberrypi-4.19.97-v7l+
export ARM_COMPILER_ARCH=aarch64-linux-gnu
# export ARM_COMPILER_PATH=${ARM_TOOL_DIR}/gcc-arm-${ARM_COMPILER_VERSION}-${ARM_COMPILER_ARCH}
export ARM_COMPILER_PATH=/usr
# export PATH=${ARM_COMPILER_PATH}/bin:$PATH