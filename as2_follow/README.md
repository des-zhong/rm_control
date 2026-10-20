# AS2 边线跟随（独立模块）

当前实现：本机读取 `/motions`，按初始化半场固定三名 RM 的身份，计算 X 均值；
狗沿固定 Y 运动，身体朝向球。两端通过 SSH 标准输入/输出通信，狗端没有 ROS 依赖。
暂不部署 YOLO，不读取深度相机，不修改比赛定位，也不做视觉补偿。

## 隔离方式

- 本次只新增 `as2_follow/`，不修改动捕、策略、避障、RM/Yanshee 控制或原启动文件。
- 每台狗运行一个本机进程和一个狗端 SDK 进程。只订阅 `/motions`，不发布任何 ROS 控制话题。
- 不随比赛自动启动，不安装系统服务，不自动起立或切换遥控模式。
- `Ctrl+C` 停止本模块，现有比赛节点不受本模块进程退出影响。
- 按用户约定，SSH 固定密码 `123` 配置在 `as2_follow/transport.py` 中，自动认证；密码不写入运行日志。IP 在 `run_follow.py`、`deploy.py` 顶部，也可用 `--ip` 覆盖。
- 连接地址：`172.20.10.13` 是下方狗，`172.20.10.14` 是上方狗。

## 坐标和初始化

| 参数 | 下方狗 `--side lower` | 上方狗 `--side upper` |
|---|---|---|
| 初始场地位置 | `(0, -2.5)` m | `(0, +2.5)` m |
| 初始身体朝向 | `+90°`（+Y） | `-90°`（-Y） |
| 固定跟随队员 | 启动时 X < -0.05 的三台 RM | 启动时 X > +0.05 的三台 RM |

`/motions.poses[0]` 是球，`[1:7]` 是 RM1..6；Yanshee 不参与本模块均值计算。
初始化要求 X 两侧各有三台 RM，成功后打印 `Locked team`；之后跨半场不会改变分组。
不满足分组条件时等待，不发送运动速度。重新运行模块会重新分组。

第一次收到有效狗端状态时，将里程计姿态 `(ox, oy, oyaw)` 对齐到参数指定的初始场地姿态；
此时狗必须已经在上表位置和朝向。**不要在运行模块后再把狗搬到启动点。**
实际位置不同时，通过 `--initial-x`、`--initial-y`、`--initial-yaw-deg` 填入实测值。
世界坐标变换为：`p_field = p_initial + R(yaw_initial - oyaw_initial) * (p_odom - p_odom_initial)`。
转向和平移均利用当前反馈姿态转换到狗的身体坐标系，允许侧向行走。

这依赖里程计，不能保证长期绝对 Y 不漂移；绝对定位改进需要外部定位或后续视觉标定。
程序重启、狗体搬动、里程计重置后均须重新摆放/标定。
场地长轴 X **仍需现场确认**；本模块不会根据现有范围常量自行认定长轴。

## 部署狗端

已在下方狗的 `/home/unitree/as2_follow/` 编译独立 SDK agent，不改动已有 SDK。
SDK 固定为官方 `unitree_sdk2` 提交 `d1abf001220d3e5f59e00a0c4aaa595e00089a4a`。
源文件依据：
- https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/example/as2/sport/as2_sport_state.cpp
- https://github.com/unitreerobotics/unitree_sdk2/blob/d1abf001220d3e5f59e00a0c4aaa595e00089a4a/include/unitree/robot/as2/sport/sport_client.hpp

首次为另一台 ARM64 狗部署（本机需 git/python3/ssh，狗端需 cmake/g++）：

```bash
# 在本仓库 src 目录执行。SDK 保持在仓库外，不加入 ROS 工作区。
git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/as2-sdk2
git -C /tmp/as2-sdk2 checkout d1abf001220d3e5f59e00a0c4aaa595e00089a4a
# 上方狗地址已确认；先用普通 ssh 确认主机指纹。
ssh unitree@172.20.10.14 exit
python3 as2_follow/deploy.py --ip 172.20.10.14 --sdk-dir /tmp/as2-sdk2
```

部署过程只上传并编译，不启动运动。更新已部署狗端的 agent：

```bash
python3 as2_follow/deploy.py --ip 172.20.10.13 --agent-only
```

两台狗分别部署、分别运行，不互相依赖。`--agent-only` 复用上次部署的 SDK。
两台狗的机内通信网卡均为 `eth10`（各自的 192.168.123.18），可通过 `--interface` 更改。

## 本机运行

首先按原流程启动动捕；本模块无需启动策略、RM 控制即可预览。
在已 source ROS 和本工作区 `install/setup.bash` 的新终端，从本仓库 `src` 目录运行：

```bash
# 默认只读：狗端不初始化运动客户端、不调用 Move/StopMove。
python3 as2_follow/run_follow.py --ip 172.20.10.13 --side lower
```

确认场地长轴、初始摆放、分组和预览结果后，实际跟随命令为：

```bash
python3 as2_follow/run_follow.py --ip 172.20.10.13 --side lower \
  --enable-motion --confirm-field-x --accept-unstamped-mocap
```

狗需事先处于可接受高级运动指令的正常站立状态。遥控/急停继续由现场人员掌握。
程序不会自动起立、解锁或关闭遥控器；SDK 请求返回错误则停止并锁定故障。
上方狗在连接及部署后使用同一命令，将 IP 替换并设置 `--side upper`。
SSH 默认自动使用固定密码 `123`，也兼容已配置的密钥认证。首次连接需手动核对并接受主机指纹。
`Ctrl+C` 关闭连接；狗端 EOF/超时会请求 StopMove，不自动趴下。

也可将新包单独编译后运行 `ros2 run as2_follow follow`，参数与上面相同。
直接运行脚本不需要重新编译其他包。其他比赛启动命令全部保持原样。

## 控制范围与停止条件

默认 X 目标限制 `[-2.3, +2.3]` m（可通过 `--xmin/--xmax` 缩小或调整）；
平移速度上限 0.12 m/s、平移加速度 0.08 m/s²，转向上限 0.25 rad/s、角加速度 0.3 rad/s²。
这些是软件限值，不代表已经验证过 AS2 的实地跟随效果。

- 本机 /motions 超过 0.25 s 无新消息，或狗端状态超过 0.25 s 未到，速度立即归零。
- 前七个动捕位姿完全不变超过 0.6 s，保守地认为可能冻结并停止；真实静止也会触发。
- 狗端 0.25 s 无有效新命令、0.3 s 无有效状态时调用 StopMove。
- 命令携带狗端最近签发的序号，排队超过约 0.2 s 或重放的旧命令不会刷新看门狗。
- 非有限数据、越界命令、SDK 错误、链路背压触发停止；部分故障锁定，需重新标定重启。
- 单次里程计平移跳变 >0.25 m、转角跳变 >0.4 rad、Y 偏离初始线 >0.3 m、
  X 超出限位 0.15 m 时，本机锁定停止。停止条件优先于加速度限制。

**现有动捕消息限制：** `/motions` 没有源帧号、源时间戳或球/刚体有效性。
原发布节点可能在无球时重复旧位置，也可能在没有合格球候选时给球填 `(0,0)`。
本模块保持原模块隔离，只能以接收超时和“数据完全不变”检测一部分故障，不能证明每帧都是新鲜、有效的动捕。
`--accept-unstamped-mocap` 表示知晓这一局限，不会修复原数据源。
后续若提供独立的带时间戳及有效性输入，可在本模块接入，无需改比赛控制链路。
狗端若 SDK 状态源时间戳为零，使用回调接收时间判断在线；日志保留源时间戳便于核对。

## 本机记录与视觉预留

每次运行独立写入 `run_data/as2_follow/<时间戳>/`（`--log-dir` 可更改）：

- `metadata.json`：IP、标定、范围和是否启用运动；不含密码。
- `telemetry.jsonl`：接收时间、原动捕位姿、分组、X 均值、狗端里程计及状态、
  场地估计姿态、身体坐标速度指令与停止原因。
- `ssh.stderr.log`：SSH 和 SDK 错误输出。

不依赖现有 `data_recorder` 的话题白名单，不修改它。
`as2_follow/vision_interface.py` 预留带采集时间的检测观测和原图/标注图数据结构。
后续视觉应独立进程、独立通道回传，避免推理或传图阻塞运动看门狗；当前无视觉后台任务。

## 验证

```bash
# 在已 source ROS + 工作区环境下：
PYTHONPATH=as2_follow:$PYTHONPATH python3 -m unittest discover -s as2_follow/tests -v
```

测试覆盖分组、坐标对齐、侧向跟随、限速、Y 修正、角度跨 ±π、里程计跳变和边界停止；
集成测试使用真实 ROS 订阅和假 SSH 对端验证均值跟随及动捕中断后的零指令，不连接实机。
狗端程序已在本机 x86_64 和下方狗 ARM64 上编译；只读检查不等于实地运动验证。

本次实机只读复查：狗端持续读到了有效状态（78 个采样中启动后的 77 个有效），
源时间戳确认为 0，错误码为 0。本机到真实 SSH agent 的预览也已验证：正常退出、
运动始终未启用、无动捕输入时指令始终为零。当前无线链路预览中出现过接收超时停止，
因此尚不能把只读连通性当作运动时延/跟随稳定性的验证。
当前自动测试共 8 项通过，其中包含断链退出、冻结动捕及动捕中断的集成检查。

## 无动捕：右侧移 + 右旋转 / Ctrl+C 测试

从 `src` 目录执行，无需 ROS、无需动捕。不要同时运行同一台狗的其他自动运动程序。
机器狗事先正常站立，右侧和转动范围留出空地，现场保留遥控急停。
这里“右”以狗自身为准：身体 Y 负向侧移、yaw 负向（俯视顺时针），身体转动时侧移方向随之转动。

```bash
# 只读检查，不发运动 API
python3 as2_follow/test_right.py --ip 172.20.10.13

# 实际运动：按 Ctrl+C 可提前停止，否则运动 5 秒后自动停止
python3 as2_follow/test_right.py --ip 172.20.10.13 --execute
```

侧移速度最多 0.03 m/s，右转角速度最多 0.06 rad/s（约 3.4°/s），2 秒线性加速。
`--seconds 3` 可缩短试验；不允许超过 10 秒。测试不使用 /motions 或场地坐标，
不需要球和球员，也不会自动起立。状态丢失/无效、SDK 错误立即结束试验，不自动恢复。

Ctrl+C（SIGINT）或 SIGTERM 到达本机脚本时，它发送独立 `STOP` 控制报文并关闭 SSH 输入；
狗端优先处理 STOP，跳过本轮运动调用，执行 StopMove 后回传结果。
即使 STOP 报文因断网未送达，狗端仍会在约 0.25 秒没有有效新指令后调用 StopMove；
EOF 也触发停止。退出时 StopMove 最多尝试三次。

终端出现 `StopMove returned success` 仅说明 SDK 返回成功，**必须现场观察是否停稳**；
未收到成功回执时脚本明确提示使用遥控急停。该软件链路无法保证断电、控制器故障等情况下的物理停车。
测试记录单独保存在 `run_data/as2_right_test/<时间戳>/`。

停止路径测试使用真实 agent 源文件加模拟 SDK 编译，验证本机脚本 SIGINT、STOP 报文、
看门狗超时和 EOF 的停止调用，不向真实机器狗发送运动指令。

### 无动捕测试的无线卡顿处理

本机测试脚本对短暂的状态回传卡顿不再立即退出：默认最长等待 1 秒，
可用 `--link-timeout` 在 0.5–2 秒间设置；超过 0.25 秒时打印 `State link delayed`。
卡顿期间本机不发送新速度，狗端的 0.25 秒指令看门狗保持原样，仍会请求停止。
状态恢复后，未达到总测试时间则继续；持续中断超过等待上限则结束。
此修改只针对无动捕测试，不改变比赛跟随桥的超时配置。

狗端状态新增 `accepted_token`、`active`、`applied_body`、`sdk_call_ms`，
用于区分“本机写出了速度”和“狗端实际以该速度调用 SDK 且返回成功”。
`Remote SDK called with nonzero velocity` 仍不代表已验证物理位移。
回归测试额外模拟了 0.4 秒状态延迟，验证能恢复并最终停止。

## 第二台狗（上方，172.20.10.14）

已在 `/home/unitree/as2_follow/` 部署同一版本的程序。每台狗在本机使用独立终端运行，
日志目录按会话区分。无动捕的实际运动测试（默认五秒；Ctrl+C 提前停止）：

```bash
python3 as2_follow/test_right.py --ip 172.20.10.14 --execute
```

恢复动捕后的上方狗预览（初始化位置 `(0, +2.5)`，朝向 -Y）：

```bash
python3 as2_follow/run_follow.py --ip 172.20.10.14 --side upper
```

启用上方狗跟随时，在上述预览命令后增加
`--enable-motion --confirm-field-x --accept-unstamped-mocap`，并按前文完成场地坐标核对。


## 两台狗统一启动（恢复动捕后）

原比赛节点、动捕和记录器准备完成，且六台 RM 仍在初始半场位置时，
在启动 RM 比赛运动之前，从 `src` 目录的新终端运行：

```bash
python3 as2_follow/run_both.py --enable-motion --confirm-field-x --accept-unstamped-mocap
```

默认下方 `172.20.10.13`、上方 `172.20.10.14`，初始位置和朝向沿用前文。
自动使用固定密码 `123`，无需输入；省略后三个运动相关参数即为两台狗只读预览。
分别确认两个节点打印 `Locked team` 后，再启动 `ep_control.launch.py`。
统一入口的 Ctrl+C 同时通知两个子进程退出，各自结束 SSH 控制；一台故障时另一台继续运行。
日志按 `lower/`、`upper/` 分目录。完整执行顺序见 [主 README](../readme.md)。
