# tools

## ros2_ci_deploy.sh

统一编译、打包、部署脚本。

### 1) 本地编译

./tools/ros2_ci_deploy.sh build --up-to "cangyi_bringup"

### 2) 打包 install

./tools/ros2_ci_deploy.sh pack --name cangyi_release

输出: deploy/artifacts/cangyi_release.tar.gz

### 3) 发布到目标机

./tools/ros2_ci_deploy.sh deploy \
	--host 192.168.1.10 \
	--user robot \
	--dest /opt/cangyirobot \
	--name cangyi_release

### 4) 一步完成

./tools/ros2_ci_deploy.sh all \
	--up-to "cangyi_bringup" \
	--host 192.168.1.10 \
	--user robot \
	--dest /opt/cangyirobot \
	--name cangyi_release
