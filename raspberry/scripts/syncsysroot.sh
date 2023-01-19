###
 # @Author Youbiao He hybtalented@163.com
 # @Date 2022-08-27
 # @LastEditors Youbiao He
 # @LastEditTime 2022-08-28
 # @FilePath /raspberry/scripts/syncsysroot.sh
 # @Description 
### 
SSH_ADDRESS=ubuntu@192.168.31.74
rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/lib sysroot
rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/usr/include sysroot/usr
rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/usr/lib sysroot/usr
rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/opt/ros sysroot/opt
# rsync -avz --rsync-path="sudo rsync" --delete $SSH_ADDRESS:/opt/vc sysroot/opt
