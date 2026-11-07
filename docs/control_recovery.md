# WASD 无法运动：本次现场诊断与修复

**当前配置（按用户最新要求）**：障碍距离自动停车已关闭，不再使用 45/15/10 cm 阈值；前后距离仅供显示。WASD 和按钮速度为 0.10 m/s。断连、松键、空格、失焦停车仍有效。以下阈值记录是故障排查时的历史情况。

本次并非仅有键盘映射问题，实际发现两种独立原因：

1. 底盘驱动报 `SerialException: device reports readiness to read but returned no data`，required 节点退出让整个 sensors.launch 关闭。相机、HTTP 数据桥仍存活，旧页面把缓存的 enabled 状态误当成车辆在线。
2. 服务恢复后，W 指令已进入控制器，但前方回波约 0.415 m，小于现有 0.45 m 停车阈值，被正确拦截。后来检测到 0.08 m/s 输出，底盘反馈约 0.079–0.086 m/s，并收到左右转向指令。

## 使用

刷新 http://127.0.0.1:8080/ ，等待控制服务在线及静止 IMU 标定结束，勾选使能。焦点在页面内时按住 W/S 前进/后退，配合 A/D 转向。A/D 单独按不会原地旋转；空格停车并取消使能。松开 W/S 停车，失焦/切换标签页取消使能。

状态栏现在显示阻断原因、下发速度、轮速以及前后最近回波。0.45 m 是先前宽角度扇区最近回波的保守阈值，不是完整车辆扫掠体碰撞检测，也不是距保险杠的严格净距离。原厂外参仍需实测确认，该距离限制现已按用户要求关闭。

## 断连处理

- 页面与 API 检查 control、wheel、原始 scan 心跳；HTTP 可达不代表底盘可用。
- 控制器独立检查轮速和雷达新鲜度，20 Hz 发布速度，指令超过 350 ms 未更新则发零速。
- 页面检测控制器 session 改变时取消使能，恢复后需重新勾选。
- API 拒绝离线时的非零请求，停车请求仍可发送；界面显示请求失败。
- 用户级 systemd 服务监督整个 sensors.launch。required 节点崩溃可能使 roslaunch 返回 0，因此使用 Restart=always，间隔 5 秒恢复，并重新静止标定。仅显式启动，未设置开机自动启用运动。

车辆上的管理命令：

```bash
systemctl --user status wheeltec-lab-sensors
systemctl --user show wheeltec-lab-sensors -p NRestarts -p MainPID
systemctl --user restart wheeltec-lab-sensors
systemctl --user stop wheeltec-lab-sensors
journalctl -u user@1000.service --since '10 minutes ago'
```

当前服务单元位于 `~/.config/systemd/user/wheeltec-lab-sensors.service`；源文件是本项目 `config/wheeltec-lab-sensors.service`。服务运行时不要额外启动第二套 sensors.launch，以免争抢串口。

## 尚未消除的硬件问题

车辆内核日志（车辆时区 CST，2026-11-07 15:41 与 15:44 左右）记录 USB 集线器 `1-2.1` 多次 `clear tt ... error -71`、断连和重新枚举，底盘串口 `1-2.1.3` 随之掉线。这是底层 USB 通信故障的证据，尚不能仅靠日志区分线缆、接头、集线器供电、干扰或驱动问题。

底盘串口自身 power/control 为 on，上游集线器为 auto；检查时均 active，因此没有证据认定休眠就是原因。未修改 USB 电源策略。应检查底盘 USB 线/集线器供电与接触，必要时在现场换短线或独立端口验证。

自动恢复不等于修好 USB；主机失联时的底盘固件独立超时停车能力尚未实测。软件看门狗通过活跃 ROS 发布零速，不能保证串口已物理断开时零速能到达 MCU。

## 验证边界

Python 回归覆盖运动学和底盘/雷达/控制失联；Node 回归覆盖 WASD、松键、空格、未使能与陈旧状态。现场观察到真实轮速与转向响应。未在有人驾驶时人为拔线或杀进程做故障注入；自动恢复配置已加载，但不要把这一点表述为通过了拔线停车测试。

更新：随后车辆日志在 15:59:33 CST 再次出现同一 USB Hub 的 error -71 和断连。用户级服务 NRestarts=2，恢复后实测 wheel/control/scan 心跳恢复、轮速为 0。证明自动重启曾实际触发，但仍不能证明底盘在物理失联期间的固件停车性能。
