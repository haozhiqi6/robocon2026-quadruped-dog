# RoboDog — 四足机器人控制软件

面向 ROBOCON 竞赛开发的四足机器人（串联腿 / 轮足）控制软件，基于 ROS 2 构建，运行于 Orange Pi 嵌入式平台。
项目实现了从底层电机驱动、传感器解算到上层步态规划与力控的完整控制链路，核心算法全部自研，C++ 代码约 8000 行。

![机器狗](doc/image.png)

## 功能特性

- **双模式运动**：步态行走与轮足模式，手柄一键切换
- **完整运动学**：串联腿正解 / 逆解、雅可比矩阵、速度与加速度映射
- **轨迹规划**：贝塞尔曲线足端轨迹 + 二次规划最优速度曲线，动态规划启发式落脚点
- **两套力控方案**：WBC 全身控制（OSQP 求解带摩擦锥约束的力分配）与 VMC 虚拟模型控制
- **前馈力矩补偿**：拉格朗日法 + 牛顿-欧拉递推，含传动系统减速比与效率换算
- **状态估计**：MPU9250 姿态解算（Mahony 互补滤波）、卡尔曼滤波、足端里程计
- **多传感器融合**：IMU、激光雷达点云、视觉巡线（八邻域边线提取）
- **实时性优化**：轨迹计算与主控制循环双缓冲分离，5 ms 控制周期

## 系统架构

软件按「决策层 — 算法层 — 硬件层」三层解耦，各节点通过 ROS 2 话题与服务通信。

```
                     ┌──────────────────────────────────────┐
  手柄 ──/joy──►     │  handle          决策层               │
                     │  按键 / 摇杆解析                      │
                     └────/joy_bing ──/bing_angle────────────┘
                                  │
                                  ▼
  MPU9250 ──/imu──►  ┌──────────────────────────────────────┐
                     │  leg_control     算法层               │
                     │  状态机 → 逆运动学 → 轨迹 → 力控       │
                     └──┬────────────────────────────┬──────┘
                        │ srv: motor_control         │ srv: footmotor_control
                        ▼                            ▼
                  ┌───────────┐              ┌────────────────┐
                  │  motor    │  硬件层       │  foot_control  │
                  │  4×2 电机  │              │  足端电机板     │
                  └─────┬─────┘              └───────┬────────┘
                        │ 串口 /dev/ttyUSB0-3        │ 串口 9600
                        ▼                            ▼
                  宇树 GO-M8010-6 × 8          足端驱动板

  激光雷达 ──► lslidar（独立工作空间）──► 点云 / SLAM
  摄像头   ──► camera  ──/camera──────► 巡线信息
```

## 目录结构

```
robocon/
├── leg_control/       核心控制节点：状态机、运动学、轨迹规划、WBC / VMC
│   ├── config/        控制参数（质量、腿长、弹簧阻尼、滤波器噪声）
│   ├── include/       头文件
│   ├── launch/        启动文件
│   └── src/           实现
├── motor/             电机驱动：四路独立串口线程，力位混合控制
├── motor_interface/   服务接口定义（MotorCtl.srv / FootMotor.srv）
├── mpu9250/           IMU 驱动：姿态解算，输出 sensor_msgs/Imu
├── handle/            手柄解析：按键 + 摇杆 → 速度 / 姿态 / 模式指令
├── foot_control/      足端电机通信节点
├── camera/            视觉巡线：八邻域边线生长与中线提取
├── lslidar/           镭神激光雷达驱动（独立 colcon 工作空间）
├── doc/               技术文档与开发日志
├── launch.sh          一键启动全部节点
├── handle.sh          仅启动手柄调试
├── swmotor.sh         刷写电机固件
└── del.sh             清理编译产物
```

## 硬件平台

| 部件 | 型号 / 说明 |
| --- | --- |
| 主控 | Orange Pi（`orangepi@192.168.31.102`） |
| 执行器 | 宇树 GO-M8010-6 电机 × 8（四腿各两台，4 Mbps 串口） |
| 姿态传感 | MPU9250 九轴 IMU（串口 115200） |
| 雷达 | 镭神 LSLIDAR（LSM10 / LSN10 系列，串口 `/dev/ttyS6` 或网口） |
| 视觉 | USB 摄像头 |
| 通信 | 腿部电机 `/dev/ttyUSB0-3`，足端电机板 9600 bps，自定义帧（0xAA 帧头 + 校验和） |

腿部编号约定（`motor_interface/srv/MotorCtl.srv`）：

| ID | 位置 |
| --- | --- |
| 0 | 前左 frontleft |
| 1 | 后右 rearright |
| 2 | 前右 frontright |
| 3 | 后左 rearleft |

## 环境依赖

- Ubuntu + ROS 2（rclcpp / ament_cmake）
- C++17
- Eigen3
- OSQP 与 OsqpEigen（WBC 二次规划求解）
- OpenCV（视觉巡线）
- 消息包：`std_msgs`、`sensor_msgs`、`joy`

## 编译

主工作空间（不含雷达驱动）：

```bash
cd ~/robocon
colcon build
source install/setup.bash
```

雷达驱动为独立工作空间，需单独编译：

```bash
cd ~/robocon/lslidar
colcon build
source install/setup.bash
```

清理编译产物：

```bash
./del.sh          # 等价于 rm -rf build log install
```

## 运行

一键启动全部节点：

```bash
./launch.sh
```

启动脚本会依次完成：IMU 初始化与水平校准（**请保持机器狗静止 5 秒**）→ 手柄节点 → 电机节点 → 足端电机节点 → 腿部控制节点，全部输出同时写入 `run.log`。

启动后终端会提示选择控制模式：

```
[0] 无力矩模式
[1] WBC 模式
[2] VMC 模式
```

单独调试手柄：

```bash
./handle.sh
```

查看雷达点云（需先启动雷达驱动）：

```bash
ros2 launch lslidar_driver viewer_scan_launch.py
```

按 `ESC` 可安全退出控制节点。

## 手柄操作

| 操作 | 功能 |
| --- | --- |
| 左摇杆 | 机身前后 / 横向速度（上限 0.5 m/s） |
| 右摇杆 | 机身俯仰 / 横滚姿态（±0.26 rad） |
| 扳机键 | 调节机身高度（0.15 – 0.35 m，默认 0.30 m） |
| A 键 | 切换至步态模式 |
| Y 键 | 切换至轮足模式 |
| LB / RB | 站起 / 蹲下 |
| Start | 电控总开关 |

## ROS 接口

### 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | 官方手柄驱动原始数据 |
| `/joy_bing` | `std_msgs/Int32MultiArray` | 按键指令：`[按键, 电控开关, 站起, 蹲下]` |
| `/bing_angle` | `std_msgs/Float32MultiArray` | 摇杆指令：`[左右, 前后, 姿态左右, 姿态上下, 油门, 刹车]` |
| `/imu` | `sensor_msgs/Imu` | IMU 姿态与角速度 |
| `/camera` | `camera/Camera` | 巡线方向（前进 / 左 / 右 / 踱步） |
| `/machine_id` | `std_msgs/Int32` | 下位机 ID |
| `/machine_angle` | `std_msgs/Float32MultiArray` | 下位机角度反馈 |

### 服务

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/motor_control` | `motor_interface/MotorCtl` | 八台电机的角度 / 速度 / Kp / Kd / 力矩下发与状态回传 |
| `/footmotor_control` | `motor_interface/FootMotor` | 足端电机速度指令 |

## 参数配置

控制参数集中在 `leg_control/config/attitude.yaml`，通过 launch 文件加载：

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `mass` | 机身质量 (kg) | 12.0 |
| `length` / `width` | 机身长 / 宽 (m) | 0.60 / 0.40 |
| `weight_S` / `weight_W` / `weight_U` | WBC 动力学 / 力矩最小化 / 力矩平滑权重 | 100 / 1 / 10 |
| `kp` / `kd_weight` | 角加速度 PD 增益 | 10.0 / 3.0 |
| `Kp_z` / `Kd_z` | 高度方向弹簧阻尼 | 1500 / 50 |
| `Kp_pitch` / `Kp_roll` | 俯仰 / 横滚扭转刚度 | 100 / 100 |
| `imu_process_noise_*` | 状态估计噪声参数 | 0.1 |

腿部几何与质量参数在 `leg_control/include/leg_control/paramter.hpp` 中定义（大腿 0.2 m、小腿 0.2 m，传动比 −2.35）。

## 开发文档

详细的技术说明、设计思路与开发日志见 [`doc/README.md`](doc/README.md)。

## 已知问题与待办

- 视觉节点目前仅发布空消息，尚未接入真实图像处理流水线
- 雷达驱动位于独立工作空间，需单独编译与 source
- 调试期存在较密集的标准输出，长时间运行会持续写入 `run.log`，建议按日志级别降噪
- 仓库尚未纳入版本控制，目前依赖定期打包备份

## 备份方式

```bash
tar -zcvf robocon.tar.gz /home/orangepi/robocon
scp orangepi@192.168.31.102:/home/orangepi/$(date "+%Y-%m-%d").tar.gz .
```

建议打包时排除 `build/ install/ log/`，可显著减小体积。
