# 基于 RRT-Connect 的 PiPER 机械臂路径规划与抓取

深蓝学院《机器人抓取与操作》Project 2 的实践项目。项目使用 MuJoCo 仿真六自由度 PiPER 机械臂，结合关节空间路径规划、逆运动学和夹爪控制，演示从初始姿态移动到罐头上方、下降、闭合夹爪和上抬的流程。

本项目基于课程提供的代码与场景进行改进。`rrt_piper_original.py` 保留了课程脚本的文件名，但其中的规划器现已改为 **RRT-Connect**；`rrt_piper_advance.py` 则在抓取场景中使用相同的双树规划方法。

## 我的核心工作

- **双向规划**：起点树与终点树交替扩展，另一棵树持续向新节点连接。
- **整段碰撞检测**：按 `path_resolution` 对关节空间边插值，在 MuJoCo 中逐点检查；抓取场景仅允许夹爪指尖与罐头的接触。
- **路径缩短**：对规划结果尝试直连，仅在整段无碰撞时删除绕路节点。
- **抓取执行**：IK 求预抓取关节角，随后执行路径跟踪、下降、夹爪闭合与上抬，并通过双相机录制视频。
- **多轮目标更新**：每轮重置场景、改变罐头位置，将世界坐标转换到机械臂基座坐标，并重新求 IK 和规划路径。

```text
罐头世界位置 → 机械臂坐标系中的目标位姿 → IK 目标关节角
          → RRT-Connect 路径 → 下降 → 夹爪闭合 → 上抬
```

## 环境

在 WSL Ubuntu 20.04、Python 3.8.10 上验证过以下版本：MuJoCo 2.3.7、dm-control 1.0.14、ikpy 3.4.2、NumPy 1.24.4、OpenCV 4.8.1。

```bash
conda create -n piper-rrt python=3.8
conda activate piper-rrt
pip install -r requirements.txt
```

请在本 README 所在目录运行脚本，因为代码使用相对路径加载 `assets/`。

## 运行

```bash
# RRT-Connect 路径规划示例，输出 rrt_robot_motion_connect.mp4
python rrt_piper_original.py

# 单次罐头抓取，输出 rrt_robot_motion_2.mp4
python rrt_piper_advance.py

# 两轮不同物体位置的 IK 与路径规划，不执行仿真或生成视频
python rrt_piper_advance.py --episodes 2 --plan-only

# 两轮抓取，分别生成带轮次编号的视频
python rrt_piper_advance.py --episodes 2 --seed 0
```

多轮模式的第 0 轮使用场景原始罐头位置，之后各轮在原位置附近 ±3 cm 随机采样。IK 或规划失败时会打印对应状态并继续下一轮。可查看仓库中的[单次抓取演示视频](rrt_robot_motion_2.mp4)。

## 代码结构

| 文件 | 用途 |
| --- | --- |
| `rrt_piper_original.py` | RRT-Connect 规划、碰撞检测、路径缩短与规划时间/路径长度输出 |
| `rrt_piper_advance.py` | 结合 IK 的抓取流程与多轮目标更新 |
| `ik.py` | 课程提供的随机目标 IK 演示 |
| `get_data.py` | 课程提供的静态图像、深度、掩码采集参考代码；尚未接入抓取流程 |
| `assets/` | MuJoCo 场景、PiPER 模型及罐头模型 |

## 当前范围

双树规划和抓取执行已接入仿真。`--episodes` 可重复求 IK、规划与录制演示视频，但**尚未实现逐帧状态/图像数据集保存、抓取成功标签或多物体场景**。当前也没有按固定随机种子批量统计原始 RRT 与 RRT-Connect 的成功率或耗时，因此不提供未经测量的百分比结论。

## 许可与致谢

保留课程项目随附的 [MIT License](LICENSE) 及原作者署名。PiPER 场景、模型和课程脚本来自课程提供的 Project 2 资料；本仓库记录在此基础上的算法与抓取流程改动。第三方模型资源应遵循其各自的原始授权。
