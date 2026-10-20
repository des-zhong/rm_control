# 运行数据可视化

无需 ROS 环境，仅依赖 matplotlib：

```bash
python3 -m pip install matplotlib
python3 run_data/visualize.py 20261012_153000_123456
```

也可以输入会话文件夹的完整路径：

```bash
python3 run_data/visualize.py /path/to/run_data/20261012_153000_123456
```

PNG 图片直接保存在输入的会话文件夹内；重复执行覆盖同名图片。
建议停止记录后运行。脚本只读取 CSV/JSONL，不修改原始数据，不需要解码 bag。
缺失文件或未收到某类话题时跳过对应图片，损坏的 JSON 行会提示并跳过。

生成图片（取决于实际记录的数据）：

- `trajectories.png`：球及机器人平面轨迹，圆点为起点、叉号为终点。
- `positions.png`、`velocities_raw.png`：位置、朝向及原始速度随时间变化。
- `ball_speed_estimated.png`：从球位置和接收时间差分估计的速度，未滤波。
- `cmd_vel.png`、`ctrl.png`、`rvo2.png`：底盘指令、策略控制量、避障输出。
- `targets.png`：策略目标位置及朝向。
- `grip_evidence.png`：视觉 caught、confidence、caught_score。
- `arm_positions.png`：机械臂位置。
- `execution_states.png`、`command_codes.png`、`ys_commands.png`：执行状态和动作请求。
- `events_001.png` 等：按每页最多 35 条分页呈现阶段事件表。

所有时间图共享同一单调接收时钟起点。轨迹名称沿用记录器的索引约定。
原始速度保留现有动捕缩放；位置差分速度也可能受定位跳变、重复旧帧、接收延迟影响。
事件图表示请求和反馈变化，不能直接当作实际抓取、踢球或进球成功的标注。
图像使用英文标签以避免缺少中文字体。抓取摄像头原始图像仍在 bag 中，本脚本不提取。
