# wheeltec-RTAB-SLAM

面向 WHEELTEC 小车的低算力 RTAB-Map 实验系统。项目在车载 Jetson Orin Nano 上融合轮速、IMU、M10P 二维激光雷达与 Astra-S RGB-D 相机，完成二维建图、闭环检测和近场三维点云生成；操作端通过 SSH 隧道在本机浏览器中查看地图、管理地图会话并遥控车辆。

> 当前参数针对一台已完成设备规则和厂家 ROS 工作空间配置的实车。默认车端地址为 `wheeltec@192.168.10.101`，ROS 版本为 Noetic，RTAB-Map 接口版本为 0.21.13。更换车辆、传感器安装位置或网络后，必须重新确认串口别名、相机标定、外参和车端路径。

## 系统组成

- `launch/`：底盘/雷达、相机和 RTAB-Map 的 ROS 启动文件。
- `config/`：EKF、RTAB-Map、相机标定及用户级 systemd 服务配置。
- `scripts/`：部署、启动、IMU 零偏估计、RGB-D 预处理、控制保护、地图会话和浏览器数据桥。
- `web/`：本机实验台页面，提供状态、地图、视频和手动驾驶界面。
- `tests/`：控制、地图会话、键盘、视频开关和投影逻辑回归测试。
- `docs/`：实车操作记录、设计边界和 RTAB-Map 原理说明。

第三方源码、下载的论文/参考材料、地图数据库、录包和实验结果不属于本仓库跟踪内容。

## 硬件与软件前提

车端需要具备以下环境：

- Ubuntu 20.04、ROS Noetic 和 catkin 工作空间；
- WHEELTEC 厂家工作空间 `/home/wheeltec/wheeltec_robot`；
- RTAB-Map/`rtabmap_ros`、`robot_localization`、`cv_bridge`、OpenCV、NumPy；
- Astra 相机驱动、M10P 雷达驱动和 GStreamer H.264/x264 组件；
- 设备别名 `/dev/wheeltec_controller` 和 `/dev/wheeltec_lidar`；
- Python 3，以及车端与本机的 `websockets` Python 包；
- 本机可通过 SSH 登录车辆，并具有 `ssh`、`tar`、Python 3 和现代浏览器。

首次使用前应先验证厂家底盘、雷达和相机驱动能够独立工作。本项目不会自动安装厂家驱动，也不会修改系统级 ROS 包。

## 首次部署

在本机项目根目录执行：

```bash
bash scripts/deploy.sh
```

如车辆地址或用户名不同，将 SSH 目标作为参数传入：

```bash
bash scripts/deploy.sh user@vehicle-ip
```

该脚本会把项目部署到车端 `~/wheeltec_slam_ws/src/wheeltec_rtabmap_lab`，并以 `-j2 -l2` 编译，构建日志写入 `~/wheeltec_slam_runtime/logs/build.log`。

首次部署还需在车端安装用户级服务单元：

```bash
ssh wheeltec@192.168.10.101
mkdir -p ~/.config/systemd/user
cp ~/wheeltec_slam_ws/src/wheeltec_rtabmap_lab/config/wheeltec-lab-*.service ~/.config/systemd/user/
systemctl --user daemon-reload
```

这些服务故意没有设置开机自动启动，避免无人值守时启用车辆运动能力。后续更新代码只需重新运行部署脚本；若服务文件有改动，再复制服务文件并执行 `daemon-reload`。

## 日常操作

### 1. 启动实验台

车辆上电并接入网络后，在本机项目根目录执行：

```bash
bash scripts/start_lab.sh
```

也可以指定其他 SSH 目标：

```bash
bash scripts/start_lab.sh user@vehicle-ip
```

脚本会建立彼此独立的数据和控制 SSH 通道，启动车端四项服务、本机静态页面和控制中继。浏览器入口为：

```text
http://127.0.0.1:8080/
```

不要直接用 `file://` 打开 `web/index.html`。页面仅监听本机回环地址，远程访问应自行配置受控的安全入口，不要直接暴露车端端口。

### 2. 新建地图

1. 等待页面显示底盘、雷达、RGB-D 相机和控制链路在线。
2. 保持车辆完全静止至少约 6 秒，等待启动时 IMU 零偏估计完成。
3. 点击“新建地图并开始”，继续静止等待状态变为“建图就绪”。
4. 勾选行驶使能后，使用 `W/S` 前后移动、`A/D` 配合转向；按空格立即停车并取消使能。
5. 低速平稳行驶，尽量让相机前方 0.5–2 m 内存在有纹理物体，并在路线中适当回到已走过区域以产生闭环候选。
6. 结束时先按空格停车，再填写名称并点击“保存当前地图”。

相机预览和特征画面可按需开启；关闭预览不影响车端采集和建图。

### 3. 续建已有地图

从地图列表选择一个非空地图，点击“从已保存地图继续”。车辆必须保持静止，等待旧地图匹配、IMU 校准和状态复核完成后再驾驶。零节点地图没有可供匹配的内容，不能续建；车辆重启后也不能假定沿用上次里程计原点。

### 4. 结束运行

先停车并取消行驶使能，再在车端停止服务：

```bash
systemctl --user stop wheeltec-lab-mapping wheeltec-lab-camera wheeltec-lab-sensors wheeltec-lab-master
```

本机的页面服务和 SSH 复用连接可在会话结束后关闭。地图位于车端 `~/wheeltec_slam_runtime/maps/`，不应提交到 Git。

## 实现原理

系统将连续运动估计与低频建图分开：

1. 厂家底盘节点提供轮式里程计，启动校准节点只在车辆静止时估计 IMU 陀螺仪零偏。
2. `robot_localization` 以 30 Hz 融合轮式前向/横向速度约束和校准后的 z 轴角速度，输出短时连续里程计；不使用未经标定的绝对航向，也不对加速度做位置双积分。
3. RGB-D 前端将已硬件配准的彩色与深度图同步去畸变，深度限制在 0.5–2 m，并将处理频率限制在约 5 Hz。
4. RTAB-Map 以约 1 Hz 创建关键帧，使用 ORB 视觉词袋检索历史地点，以 RGB-D 视觉几何和二维激光 ICP 验证/精化约束，再对 x、y、yaw 位姿图进行优化。
5. 车端桥接程序只向本机发送有上限的地图、点云、状态和视频数据；浏览器负责绘制，不参与 SLAM 求解。

当前主要计算约束为 400 个 ORB 特征、150 节点工作记忆、350 ms RTAB-Map 时间预算、5 cm ICP 体素和 15 次 ICP 迭代。这些是资源预算，不是硬实时或精度保证。更完整的算法说明见 [docs/rtabmap_low_compute.md](docs/rtabmap_low_compute.md)。

## 控制保护与注意事项

- 当前手动速度为 0.10 m/s，车端输入上限为 0.12 m/s；控制指令超过 350 ms、底盘/雷达数据陈旧、IMU 未完成校准或地图正在切换时，软件看门狗发布零速度。
- 雷达最近距离当前只用于显示，**没有启用自动避障停车**。操作者必须全程看护，首次运行应架空驱动轮或在空旷区域低速测试。
- 浏览器松键、失焦、空格和断开会触发停车，但软件无法保证 USB/串口物理断开时零速度一定到达底盘 MCU。不要把本系统作为功能安全控制器。
- 低电压提醒阈值为 9.9 V，连续低于阈值超过 20 秒才提示；它不会自动停车。电量不足可能导致 USB 或底盘通信异常。
- 激光雷达、相机和 IMU 外参仍包含厂家默认/暂定值。移动传感器或更换车体后必须重新测量，不能直接沿用。
- RGB 与深度必须已经在驱动层按 RGB 像素硬件配准。相机标定文件与实际设备不一致会直接影响点云和闭环几何。
- 服务运行时不要额外启动第二套 `sensors.launch`，否则会争抢串口。
- 删除地图会删除已保存副本；当前工作副本是独立的。操作前确认名称并按需在车外备份。

## 常用排障

检查车端服务：

```bash
systemctl --user status wheeltec-lab-master wheeltec-lab-sensors wheeltec-lab-camera wheeltec-lab-mapping
journalctl --user -u wheeltec-lab-sensors --since "10 minutes ago"
```

若页面能打开但车辆不能运动，依次检查：行驶使能、IMU 是否完成静止校准、地图是否就绪、底盘/雷达心跳、控制链路以及电池电压。若设备反复掉线，同时检查 USB 线缆、接头、集线器供电和内核日志；服务自动重启只能恢复进程，不能修复硬件通信问题。

若部署构建失败，先查看车端日志：

```bash
tail -n 100 ~/wheeltec_slam_runtime/logs/build.log
```

更详细的现场记录见 `docs/quick_start.md`、`docs/control_recovery.md` 和 `docs/display_update.md`。

## 测试

不依赖 ROS 运行环境的回归测试可在项目根目录执行：

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
node --test tests/*.test.js
```

完整验证仍应包括实车传感器话题、TF、地图保存/续建、断线停车和低速驾驶检查。自动化测试通过不代表传感器标定或硬件安全已经得到验证。
