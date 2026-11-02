# 基于 RRT-Connect 的 PiPER 机械臂路径规划与抓取

深蓝学院《机器人抓取与操作》Project 2 的实践项目。项目使用 MuJoCo 仿真六自由度 PiPER 机械臂，结合关节空间路径规划、逆运动学和夹爪控制，演示从初始姿态移动到罐头上方、下降、闭合夹爪和上抬的流程。

本项目基于课程提供的代码与场景进行改进。`rrt_piper_original.py` 保留了课程脚本的文件名，但其中的规划器现已改为 **RRT-Connect**；`rrt_piper_advance.py` 则在抓取场景中使用相同的双树规划方法。

## 我的核心工作

### 1. RRT-Connect 双向路径规划

基于课程提供的单树 RRT 框架，重构关节空间规划器：

- 从起点和终点分别建立搜索树，交替扩展，并让另一棵树朝新节点持续连接；两树到达同一关节配置后拼接路径。
- 保留目标偏向随机采样，按 PiPER 六个关节的限位约束扩展状态。
- 对扩展边的所有插值点进行 MuJoCo 碰撞检测，避免只检查节点终点造成的漏检。
- 用带碰撞验证的 shortcut 删除绕路节点，再生成连续的关节空间轨迹。

### 2. 抓取任务流程

完成 `rrt_piper_advance.py` 中的课程 TODO，并串联路径规划、IK 和抓取执行：

- 使用 ikpy 和 PiPER URDF，将预抓取目标位姿转换为六个关节角。
- 通过关节角线性插值完成下降和上抬；控制夹爪闭合，并通过双相机渲染录制演示视频。
- 在规划阶段区分机械臂非法碰撞与夹爪指尖接触罐头的允许情况。
- 支持多轮改变罐头位置；每轮重新换算目标坐标、求 IK 并规划路径。

### 3. 调试与改进

- 修正扩展步数取整造成的目标点偏差，并检查整段路径的碰撞。
- 对规划结果做路径缩短和插值，减少不必要的关节空间绕行。
- 针对抓取时碰撞或物体位移问题，调整预抓取高度和下降高度；这些值仍可按场景继续微调。

## 技术栈

| 类别 | 技术 |
| --- | --- |
| 编程语言 | Python 3.8 |
| 物理仿真 | MuJoCo 2.3.7、dm-control 1.0.14 |
| 机器人学 | ikpy 3.4.2、URDF、transformations |
| 数值计算与视觉 | NumPy、SciPy、OpenCV、Pillow（PIL） |

## 项目成果

1. 实现 RRT-Connect 双树搜索、整段碰撞检测和路径缩短。**根据作者提供的对比结果，相比课程原始单树 RRT，路径规划成功率提升至 98%，收敛速度提升约 37%。**
2. 将 IK、无碰撞路径规划、下降、夹爪控制、上抬与双相机录像串成抓取演示流程。
3. 支持不同罐头位置的多轮 IK 与规划，并提供单轮抓取[演示视频](rrt_robot_motion_2.mp4)。

上述百分比是项目对比结果；统计脚本、测试次数、场景配置与原始记录尚未纳入仓库，当前无法直接复现该数值。

## 运行流程

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

多轮模式的第 0 轮使用场景原始罐头位置，之后各轮在原位置附近 ±3 cm 随机采样。IK 或规划失败时会打印对应状态并继续下一轮。

## 代码结构

| 文件 | 用途 |
| --- | --- |
| `rrt_piper_original.py` | RRT-Connect 规划、碰撞检测、路径缩短与规划时间/路径长度输出 |
| `rrt_piper_advance.py` | 结合 IK 的抓取流程与多轮目标更新 |
| `ik.py` | 课程提供的随机目标 IK 演示 |
| `get_data.py` | 课程提供的静态图像、深度、掩码采集参考代码；尚未接入抓取流程 |
| `assets/` | MuJoCo 场景、PiPER 模型及罐头模型 |

## 当前范围

双树规划和抓取执行已接入仿真。`--episodes` 可重复求 IK、规划与录制演示视频，但**尚未实现逐帧状态/图像数据集保存、抓取成功标签或多物体场景**。仓库目前也没有可独立复现上述性能百分比的基准测试资料。

## 许可与致谢

保留课程项目随附的 [MIT License](LICENSE) 及原作者署名。PiPER 场景、模型和课程脚本来自课程提供的 Project 2 资料；本仓库记录在此基础上的算法与抓取流程改动。第三方模型资源应遵循其各自的原始授权。
