# deploy
放 systemd、Docker、OTA 升级脚本和部署模板。

## 推荐流程（后续项目可复用）

1. 在开发机编译（按需构建到目标包）

./tools/ros2_ci_deploy.sh build --up-to "cangyi_bringup"

2. 打包 install 目录

./tools/ros2_ci_deploy.sh pack --name cangyi_release

3. 发布到目标机并解压

./tools/ros2_ci_deploy.sh deploy \
	--host 192.168.1.10 \
	--user robot \
	--dest /opt/cangyirobot \
	--name cangyi_release

4. 目标机手动启动（验证阶段）

source /opt/cangyirobot/install/setup.bash
ros2 launch cangyi_bringup bringup.launch.py mode:=control can_port:=can0 arm_type:=piper

## systemd 常驻

- 启动脚本: deploy/scripts/start_cangyi_bringup.sh
- 服务模板: deploy/systemd/cangyi_bringup.service.template

使用方法:

1) 把模板中的 <robot_user>、<workspace_dir> 替换为实际值。
2) 保存为 /etc/systemd/system/cangyi_bringup.service。
3) 执行:

sudo systemctl daemon-reload
sudo systemctl enable cangyi_bringup
sudo systemctl restart cangyi_bringup
sudo systemctl status cangyi_bringup