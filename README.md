# mocap_ws — FZMotion → ROS 2 Jazzy

把 FZMotion（LuMoSDK，ZMQ + protobuf）的刚体位姿、速度和加速度发布为 ROS 2 topic 与 TF。
当前适配 **Ubuntu 24.04 / ROS 2 Jazzy / 系统 Python 3.12**。
原来的 Humble/Python 3.10 `env/` 与 `build/ install/ log/` 保留，Jazzy 不使用它们。

## 1. 安装、编译和启动

系统需要已有 `/opt/ros/jazzy`。缺少构建依赖时安装：

```bash
sudo apt install python3-pip python3-venv python3-colcon-common-extensions \
  ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-tf2-ros \
  ros-jazzy-ros2launch ros-jazzy-rmw-cyclonedds-cpp
```

使用没有 source Humble 的新终端：

```bash
cd ~/mocap_ws
./setup_jazzy.sh
./run.sh
```

`setup_jazzy.sh` 用 `/usr/bin/python3` 创建 `env_jazzy/`（包含系统 site-packages），
只在此虚拟环境安装 ZMQ/protobuf；编译输出位于 `build_jazzy/ install_jazzy/ log_jazzy/`。
可重复运行。不要把旧 Python 3.10 虚拟环境直接拿来运行 Jazzy，亦不要使用 Conda Python。
这遵循 [ROS 官方的 Python 解释器兼容要求](https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Using-Python-Packages.rst)。

`run.sh` 默认读取 `src/fzmotion_ros/config/fzmotion.yaml`，命令行参数优先：

```bash
./run.sh --ros-args -p ip:=169.254.209.149 -p rigid_name:=main
./run.sh --ros-args -p yaw_offset_deg:=90.0
# 也可以选其他 YAML：
FZMOTION_CONFIG=/absolute/path/to/config.yaml ./run.sh
```

标准 ROS 启动方式（协议模块已随包安装，无需手动设置 `PYTHONPATH`）：

```bash
source ./source_jazzy.sh
ros2 launch fzmotion_ros fzmotion.launch.py
# 自定义 YAML：
ros2 launch fzmotion_ros fzmotion.launch.py params_file:=/absolute/path/to/config.yaml
# 或直接运行节点（不加 --params-file 时使用代码默认参数）：
ros2 run fzmotion_ros fzmotion_node --ros-args -p ip:=127.0.0.1 -p port:=6899
```

`source_jazzy.sh` 在未设置时默认 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、
`ROS_DOMAIN_ID=0`；保留用户已设置的值。每个查看 topic 的终端也需要 source 此脚本。
`setup_jazzy.sh` 构建出的节点入口使用工作区 Python 3.12 虚拟环境。

```
mocap_ws/
├── env_jazzy/                  # Python 3.12，工作区内依赖
├── py/                         # SDK 示例、原始帧工具和 mock 数据源
├── src/fzmotion_ros/
│   ├── fzmotion_ros/            # 节点、坐标转换、原始 SDK protobuf 协议
│   ├── config/fzmotion.yaml
│   └── launch/fzmotion.launch.py
├── scripts/check_jazzy.py       # 无动捕设备的 ZMQ → ROS2 集成检查
├── setup_jazzy.sh
├── source_jazzy.sh
├── run.sh
└── build_jazzy/ install_jazzy/ log_jazzy/
```

---

## 2. 发布的话题

| 话题(默认) | 类型 | header.frame_id | 内容 |
|---|---|---|---|
| `/fzmotion/pose` | `geometry_msgs/PoseStamped` | `world` | 位置 + 姿态 |
| `/fzmotion/velocity` | `geometry_msgs/TwistStamped` | `world` | 线速度(`linear`) + 角速度(`angular`) |
| `/fzmotion/acceleration` | `geometry_msgs/AccelStamped` | `world` | 线加速度 + 角加速度 |
| `/tf` | `tf2` | `world`→`<rigid名>` | 刚体位姿 |

- `frame_id` 由参数 `frame_id` 决定（默认 `world`，可改 `map`）。
- 只有**刚体(RigidBody)**有速度/加速度；散点(marker)、骨骼没有这些字段。
- 多刚体时用 `rigid_name` 选（默认 `main`）。

---

## 3. 单位与坐标系（核心）

FZMotion SDK 给出的各量单位/坐标系，以及本包发布到 ROS 后的单位/坐标系：

| 量 | SDK 字段 | SDK 单位 | SDK 坐标系 | ROS 单位 | ROS 坐标系 | 缩放参数 |
|---|---|---|---|---|---|---|
| 位置 X/Y/Z | `X` `Y` `Z` | mm | FZ 世界系 | m | 世界系 | `pos_scale`=0.001 |
| 线速度 | `speeds.XfSpeed/YfSpeed/ZfSpeed` | m/s | FZ 世界系 | m/s | 世界系 | `vel_scale`=1.0 |
| 线加速度 | `acceleratedSpeeds.Xf…/Yf…/Zf…` | m/s² | FZ 世界系 | m/s² | 世界系 | `acc_scale`=1.0 |
| 角速度 | `palstance.fXPalstance/fY…/fZ…` | rad/s | **刚体自身系** | rad/s | **世界系** | `ang_scale`=1.0 |
| 角加速度 | `accpalstance.AccfX…/AccfY…/AccfZ…` | rad/s² | **刚体自身系** | rad/s² | **世界系** | `ang_scale`=1.0 |
| 姿态 | `qx,qy,qz,qw` | — | 刚体→FZ世界 | 四元数 | 世界系 | — |

关键点：

- **位置/线速度/线加速度** 在 SDK 里已经是 **FZ 世界系**，只需做轴映射（+位置 mm→m）。
- **角速度/角加速度** 在 SDK 里是 **刚体自身系(body frame)**，发布前先用刚体姿态转到世界系（`quat_rotate`），再做轴映射。因此 `/fzmotion/velocity`、`/fzmotion/acceleration` 的 `angular` 最终与 `linear` 一样是**世界系**，整个消息自洽。
- 位置 mm→m 用 `pos_scale`；速度/加速度数值已经是 SI，默认不缩放（`*_scale=1.0`）。

处理流水线（每帧）：

```
位置:       p_ros   = R_map · (p_fz_mm * pos_scale)
姿态:       q_ros   = q_map ⊗ q_fz ⊗ q_map⁻¹        (相似变换)
线速度:     v_ros   = R_map · (v_fz * vel_scale)
线加速度:   a_ros   = R_map · (a_fz * acc_scale)
角速度:     ω_ros   = R_map · quat_rotate(q_fz, ω_body * ang_scale)
角加速度:   α_ros   = R_map · quat_rotate(q_fz, α_body * ang_scale)
```

其中 `quat_rotate(q, v)` = 用刚体四元数把 body 系向量转到世界系；`R_map`/`q_map` 见下一节。

---

## 4. Y-up → ROS 坐标轴变换

- **FZMotion**：右手系，**Y 朝上**（X、Y=Up、Z）。
- **ROS（REP-103）**：x=前、y=左、z=上。

硬约束：**Y(Up) → z**（同号）。水平面两轴谁对 x、谁对 y 及符号，由"你定义哪条水平方向为前方"决定，统一用一个偏角 `θ`（`yaw_offset_deg`）覆盖：

```
R_map = Rz(θ) · Rx(+90°)
p_ros = R_map · p_fz
R_ros = R_map · R_fz · R_mapᵀ        （姿态用相似变换）
```

三种常见情况（只有 θ 不同）：

| 情况 | 你的前方 | `yaw_offset_deg` | 向量映射 `(x,y,z)_fz → ROS` |
|---|---|---|---|
| A | FZ **+X** 为前（最常见） | `0` | `(x, −z, y)` |
| B | FZ **+Z** 为前 | `+90` | `(z, x, y)` |
| C | FZ **−Z** 为前 | `−90` | `(−z, −x, y)` |

> 无论哪种，**FZ 的 Y(Up) 永远变成 ROS 的 z(上)**。

**怎么定 θ（不用猜）**：把 `main` 刚体沿你心里的"前方"推/走约 1 m，看 `/fzmotion/pose` 的 `position.x` 是否增大：
- `x` 增大 → 情况 A，`yaw_offset_deg:=0.0`（默认）
- `y` 变化、x 不动 → 前方是 FZ 的 ±Z，用 `+90` 或 `−90`
- 或用 rviz2 / `tf2_echo` 对照实物方向微调

姿态：**不要**直接搬 FZ 的欧拉角（Roll=Z/Pitch=X/Yaw=Y，是 Y-up 欧拉角）。本包用 SDK 四元数做相似变换 `q_ros = q_map ⊗ q_fz ⊗ q_map⁻¹`，与位置同一套世界旋转。

`apply_axis_map:=false` 可整体关闭轴映射（回到 FZ 原始坐标，仅用于比对）。

> 说明：绕竖直轴的旋转（如地面机器人 yaw）在 A/B/C 任一下都能正确落到 ROS 的 z 轴；`θ` 只影响水平面 x/y 的对齐。

---

## 5. 命令行：查看 ROS 话题

**终端 A —— 启动节点（保持开着）：**
```bash
cd ~/mocap_ws
./run.sh
```

**终端 B —— 查看（每条命令单独执行）：**
```bash
source ~/mocap_ws/source_jazzy.sh

# 列话题 / 看发布者
ros2 topic list
ros2 topic info /fzmotion/pose

# 看数据
ros2 topic echo /fzmotion/pose
ros2 topic echo /fzmotion/velocity        # linear=线速度  angular=角速度
ros2 topic echo /fzmotion/acceleration    # linear=线加速度 angular=角加速度

# 频率取决于发送端帧率与 poll_rate；本地 mock 为 50 Hz
ros2 topic hz /fzmotion/pose

# 看 TF 位姿
ros2 run tf2_ros tf2_echo world main
```

提示：
- `ros2 topic echo --once` 偶尔报 `does not appear to be published yet / Could not determine the type`，是 DDS 发现时序，**重跑一次**即可（尤其节点刚启动时）。
- 电脑和机器人以及所有查看 topic 的终端必须使用相同 `ROS_DOMAIN_ID` 与兼容的 QoS。可在两端启动节点前显式 `export ROS_DOMAIN_ID=0`；脚本不会覆盖已设置的 domain。
- 同一组输出 topic 只启动一个采集节点；多个发布者会导致消息混合，可用 `ros2 topic info -v /fzmotion/pose` 检查。

---

## 6. 纯 python 使用（不经 ROS）

`./setup_jazzy.sh` 完成后，直接在 `py/` 下使用 `env_jazzy` 的 Python：

```bash
cd ~/mocap_ws/py

# 看原始帧（mm、raw protobuf）
../env_jazzy/bin/python raw_dump.py 169.254.209.149

# 旋转时对比 body 系 vs world 系角速度（验证坐标约定）
../env_jazzy/bin/python diag_spin.py 169.254.209.149

# 官方示例
../env_jazzy/bin/python PythonSample.py 169.254.209.149
```

自写脚本时，`import LusterFrameStruct_pb2` / `import LuMoSDKClient` 即可（同在 `py/` 目录）。

---

## 7. 参数一览

通过 `./run.sh --ros-args -p 名字:=值` 覆盖，或在 `config/fzmotion.yaml` 里改。

| 参数 | 默认 | 说明 |
|---|---|---|
| `ip` | `169.254.209.149` | FZMotion 发送端 IP |
| `port` | `6868` | ZMQ 端口 |
| `frame_id` | `world` | 发布消息的世界系 frame 名（可 `map`） |
| `pos_scale` | `0.001` | 位置 mm→m |
| `vel_scale` | `1.0` | 线速度缩放（SDK 已是 m/s）|
| `acc_scale` | `1.0` | 线加速度缩放（m/s²）|
| `ang_scale` | `1.0` | 角速度/角加速度缩放（rad/s, rad/s²）|
| `rigid_name` | `main` | 发布哪个刚体；`""`=第一个 tracked |
| `pose_topic` | `fzmotion/pose` | 位姿话题 |
| `vel_topic` | `fzmotion/velocity` | 速度话题 |
| `accel_topic` | `fzmotion/acceleration` | 加速度话题 |
| `publish_tf` | `true` | 是否发 world→刚体 TF |
| `apply_axis_map` | `true` | Y-up→Z-up 轴映射开关 |
| `yaw_offset_deg` | `0.0` | 前方偏角 θ（0 / +90 / −90）|
| `poll_rate` | `200.0` | ZMQ 轮询频率(Hz) |

切换前方：
```bash
./run.sh --ros-args -p yaw_offset_deg:=90.0      # 情况B
./run.sh --ros-args -p yaw_offset_deg:=-90.0     # 情况C
./run.sh --ros-args -p frame_id:=map
```

---

## 8. 回归测试 / 自检

自动检查三个入口（`run.sh` / `ros2 run` / `ros2 launch`）、YAML/命令行覆盖、
位姿与速度/加速度数值、TF、同帧时间戳、跟踪丢失/坏包恢复及 Ctrl+C 正常退出：

```bash
cd ~/mocap_ws
source ./source_jazzy.sh
python scripts/check_jazzy.py
```

检查使用独立 domain（默认 91，可用 `--domain` 改）、仅 localhost 发现和随机本地 ZMQ 端口。
不连接真实动捕、不发机器人控制命令，也不修改当前终端的 domain。
自动检查通过不代表真实设备或电脑到机器人的网络已通过验收。


用已知值的 mock 数据源验证"坐标轴 + 单位换算"链路（不需要真实设备）：

```bash
# 终端1：启动 mock（端口 6899），发送已知的恒定帧
cd ~/mocap_ws/py && ../env_jazzy/bin/python mock_pub.py 6899

# 终端2：让节点连到 mock
cd ~/mocap_ws
./run.sh --ros-args -p ip:=127.0.0.1 -p port:=6899

# 终端3：查看，应精确等于期望值（见 mock_pub.py 顶部注释）
source ~/mocap_ws/source_jazzy.sh
ros2 topic echo /fzmotion/velocity --once
```

`axis_map.py` 也可单独做纯数值验证（导入 `fz_vec_to_ros` / `fz_quat_to_ros` / `quat_rotate`）。

---

## 9. 常见问题速查

| 现象 | 原因 / 处理 |
|---|---|
| 速度/加速度全 0 | 发送端 FZMotion 版本 < B004 或未勾选输出；`IsTrack=false` |
| 线速度差 1000 倍 | 误把 m/s 当成 mm/s 又乘了 0.001（本包已默认 `vel_scale=1`）|
| 方向偏 90°/180° | `yaw_offset_deg` 没设对（A/B/C 换一下）|
| 角速度方向反 | 若做**倾斜轴**旋转发现镜像 → 四元数约定相反，把 `quat_rotate` 改成共轭（`world` 里对竖直轴无影响）|
| 角加速度很抖 | 角加速度天生噪声大，别用它判断转轴，用角速度(`velocity.angular`)|
| 抖动大 | `CONFLATE` 丢帧；优先用 SDK 自带速度字段（本包已这样）|
| `does not appear to be published yet` | 节点没起 / 域不一致 / DDS 发现时序，重跑或检查 `ROS_DOMAIN_ID` |
| 频率异常叠加 | 检查 topic 的发布者数量，关闭重复启动的采集节点 |

---

## 10. 备注

- protobuf 固定 `3.20.3`，保持 SDK 原始协议字节和字段定义不变；协议文件位于 `src/fzmotion_ros/fzmotion_ros/LusterFrameStruct_pb2.py`，`py/` 下同名文件为兼容入口。FZMotion 协议升级时更新包内协议文件并重新编译。
- 角速度/角加速度已从 body 系转到世界系发布；若下游需要 **base_link/body 系**的角速度，需再乘 `R_world_body⁻¹`，可另开话题。
- 坐标换算集中在 `axis_map.py`（纯函数，无 ROS 依赖），便于单测与复用。

## 11. 电脑转发到机器人 / I-BFM 接口

- 电脑运行本工作区；机器人在 Jazzy 容器内直接订阅 `/fzmotion/pose` 和 `/fzmotion/velocity`。
  同步 domain、RMW、消息类型和 QoS，并配置能到达对端的 DDS 网卡。容器使用 host 网络。
- 保留当前可靠传输、队列深度 10 的发布 QoS；不会因迁移而改变既有订阅兼容性。
- `frame_id=world` 是坐标系名称，不会自动完成空间标定。箱子、机器人基座和目标需要统一坐标与刚体标记偏移。
- 真实设备使用 `use_sim_time=false`（默认）。电脑和机器人需同步系统时钟。
  **当前消息时间戳是电脑接收并处理该帧时的 ROS 时间**，不是动捕硬件采样时间；
  SDK 时间戳与 ROS 时钟尚未做标定。因此只看消息时间戳不能推断完整的采集延迟。
- 没有数据或 `IsTrack=false` 时不发布旧数据；损坏的 protobuf 包会跳过并告警。
  当前下游 I-BFM 默认输入时间匹配容差为 30 ms，需实测发送频率、时钟偏差和网络抖动。
- 本次只适配动捕工作区；I-BFM 的真实基座状态、接触估计和真机启用仍需另行接入。
