from dm_control import mujoco
import cv2
import numpy as np
import random
import time

# 加载控制模型
model = mujoco.Physics.from_xml_path('assets/piper_rrt.xml')

class RRTConnect:
    class Node:
        def __init__(self, q):
            self.q = np.asarray(q, dtype=float).copy()
            self.path_q = []
            self.parent = None

    def __init__(self, start, goal, joint_limits, expand_dis=0.1, path_resolution=0.01, goal_sample_rate=5, max_iter=5000):
        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.joint_limits = joint_limits
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list_start = []
        self.node_list_goal = []
        self.raw_path_length = None

    def planning(self, model):
        """Grow two trees until a collision-free connection joins them."""
        self.node_list_start = [self.start]
        self.node_list_goal = [self.end]
        if not check_collision_with_dm_control(model, self.start.q):
            return None
        if not check_collision_with_dm_control(model, self.end.q):
            return None

        for i in range(self.max_iter):
            if i % 2 == 0:
                active_tree, other_tree = self.node_list_start, self.node_list_goal
                bias_target = self.end
            else:
                active_tree, other_tree = self.node_list_goal, self.node_list_start
                bias_target = self.start

            random_node = self.get_random_node(bias_target)
            status, new_node = self.extend(active_tree, random_node, model)
            if status == "trapped":
                continue

            connect_status, other_node = self.connect(other_tree, new_node, model)
            if connect_status == "reached":
                if active_tree is self.node_list_start:
                    return self.generate_final_course(new_node, other_node, model)
                return self.generate_final_course(other_node, new_node, model)

        return None

    def extend(self, tree, target_node, model):
        nearest_ind = self.get_nearest_node_index(tree, target_node)
        new_node = self.steer(tree[nearest_ind], target_node, self.expand_dis)
        if new_node is None or not self.check_collision(new_node, model):
            return "trapped", None

        tree.append(new_node)
        if np.linalg.norm(new_node.q - target_node.q) < 1e-9:
            return "reached", new_node
        return "advanced", new_node

    def connect(self, tree, target_node, model):
        """Keep extending the other tree toward the new node."""
        while True:
            status, new_node = self.extend(tree, target_node, model)
            if status != "advanced":
                return status, new_node

    def get_nearest_node_index(self, node_list, rnd_node):
        """
        求离随机节点最近的节点的索引。
        参数:
        node_list: RRT树节点列表。
        rnd_node：随机生成的节点。
        返回:
        节点列表中最近节点的索引。
        """
        dlist = [np.linalg.norm(node.q - rnd_node.q) for node in node_list]
        min_index = dlist.index(min(dlist))
        return min_index
    
    def steer(self, from_node, to_node, extend_length=float("inf")):
        delta_q = to_node.q - from_node.q
        distance = np.linalg.norm(delta_q)
        if distance < 1e-12:
            return None

        travel = min(extend_length, distance)
        num_steps = max(1, int(np.ceil(travel / self.path_resolution)))
        new_node = self.Node(from_node.q)
        for i in range(1, num_steps + 1):
            fraction = (travel / distance) * (i / num_steps)
            new_q = from_node.q + fraction * delta_q
            new_node.path_q.append(np.clip(
                new_q,
                [lim[0] for lim in self.joint_limits],
                [lim[1] for lim in self.joint_limits],
            ))
        new_node.q = new_node.path_q[-1].copy()
        new_node.parent = from_node
        return new_node

    def get_random_node(self, bias_target):
        if random.random() * 100 >= self.goal_sample_rate:
            rand_q = [random.uniform(joint_min, joint_max) for joint_min, joint_max in self.joint_limits]
        else:
            rand_q = bias_target.q
        return self.Node(rand_q)

    def check_collision(self, node, model):
        return all(check_collision_with_dm_control(model, q) for q in node.path_q)

    def shortcut_path(self, waypoints, model):
        """Remove detours only when the direct joint-space edge is collision-free."""
        shortened = [waypoints[0]]
        current = 0
        while current < len(waypoints) - 1:
            # Try the farthest waypoint first; the existing next edge is safe.
            for candidate in range(len(waypoints) - 1, current, -1):
                if candidate == current + 1:
                    break
                edge = self.steer(waypoints[current], waypoints[candidate])
                if edge is not None and self.check_collision(edge, model):
                    break
            shortened.append(waypoints[candidate])
            current = candidate
        return shortened

    def generate_final_course(self, start_node, goal_node, model):
        start_branch = []
        node = start_node
        while node is not None:
            start_branch.append(node)
            node = node.parent
        start_branch.reverse()

        goal_branch = []
        node = goal_node
        while node is not None:
            goal_branch.append(node)
            node = node.parent

        # Both branches share the same connection configuration.
        waypoints = start_branch + goal_branch[1:]
        self.raw_path_length = sum(
            np.linalg.norm(b.q - a.q) for a, b in zip(waypoints, waypoints[1:])
        )
        waypoints = self.shortcut_path(waypoints, model)
        path = [waypoints[0].q.copy()]
        for from_node, to_node in zip(waypoints, waypoints[1:]):
            edge = self.steer(from_node, to_node)
            if edge is not None:
                path.extend(edge.path_q)
        return path


def check_collision_with_dm_control(model, joint_config):
    """
    函数使用dm_control的碰撞检测来检查给定的联合配置是否会导致碰撞。
    参数:
    模型:dm_control Mujoco模型
    joint_config:要检查碰撞的关节角度列表
    返回:
    无碰撞时为True,有碰撞时为False
    """
    model.data.qpos[:] = joint_config 
    model.forward() 

    # 检测碰撞
    contacts = model.data.ncon  # Number of contacts (collisions)
    return contacts == 0  # True if no contacts (collision-free)

def get_end_effector_pose(physics, body_name="link6"):
    """获取末端执行器位姿（位置和旋转矩阵）"""
    physics.forward()  # 必须先更新物理状态
    
    # 获取body ID
    body_id = physics.model.name2id(body_name, 'body')
    if body_id == -1:
        raise ValueError(f"未找到body: {body_name}")
    
    # 位置 (x,y,z)
    pos = physics.named.data.xpos[body_name]  # 使用named接口
    
    # 旋转矩阵 (3x3)
    rot = physics.named.data.xmat[body_name].reshape(3, 3)
    
    return pos.copy(), rot.copy()  # 返回拷贝避免后续修改

def apply_rrt_path_to_dm_control(model, path, video_name="rrt_robot_motion.mp4"):
    """
    Function to apply the RRT-generated path (list of joint configurations) to the dm_control simulation,
    while recording the frames into a video.
    
    Args:
        model: dm_control Mujoco model
        path: List of joint configurations generated by the RRT planner
        video_name: Name of the output video file
    """
    # Setup for video recording
    width, height = 640, 480  # Resolution of each camera
    fps = 20.0
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for mp4
    out = cv2.VideoWriter(video_name, fourcc, fps, (1280, 480))  # Two 640x480 images side by side
    # Match simulation time to video time (0.002 s * 25 steps = 1 / 20 s).
    steps_per_frame = max(1, round(1.0 / (fps * model.model.opt.timestep)))

    # set initial joint angles
    model.data.qpos[:] = start
    model.data.ctrl[:] = start
    model.forward()

    # Apply the path to the simulation and record the video
    for q in path:
        model.data.ctrl[:] = q  # Set joint angles
        for _ in range(steps_per_frame):
            model.step()

        # Render from both cameras and concatenate side by side
        frame_1 = model.render(camera_id=0, width=width, height=height)
        frame_2 = model.render(camera_id=2, width=width, height=height)
        frame_combined = np.concatenate((frame_1, frame_2), axis=1)
        
        # Convert frame from RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame_combined, cv2.COLOR_RGB2BGR)
        
        # Write the frame to the video
        out.write(frame_bgr)

    # Release the video writer
    out.release()
    print(f"Video saved as {video_name}")


start = [45/57.2958, 110/57.2958, -45/57.2958, 60/57.2958, -57/57.2958, 20/57.2958]   # Start joint angles
goal = [-45/57.2958, 110/57.2958, -45/57.2958, -60/57.2958, -57/57.2958, 20/57.2958]   # Goal joint angles
#joint limits
joint_limits = [[-2.618,2.618],[0,3.14158],[-2.697,0],[-1.832,1.832],[-1.22,1.22],[-3.14158,3.14158]] 

model.data.qpos[:6] = start
model.forward()

# 获取初始末端位姿
init_pos, init_rot = get_end_effector_pose(model)
print("\n初始末端位置:", np.round(init_pos, 4))
print("初始旋转矩阵:\n", np.round(init_rot, 4))

# 可选：转换为欧拉角（ZYX顺序，角度制）
from scipy.spatial.transform import Rotation as R
euler_angles = R.from_matrix(init_rot).as_euler('zyx', degrees=True)
print("初始欧拉角(度):", np.round(euler_angles, 2))

# Run RRT-Connect and report planning metrics for comparison with the original RRT.
rrt = RRTConnect(start, goal, joint_limits)
planning_start = time.perf_counter()
rrt_path = rrt.planning(model)  # Generate the RRT path
print(f"Planning time: {time.perf_counter() - planning_start:.3f} s")

# Apply the path to the MuJoCo simulation and record video
if rrt_path:
    print("Path found!")
    path_length = sum(np.linalg.norm(b - a) for a, b in zip(rrt_path, rrt_path[1:]))
    print(f"Joint-space path length: {rrt.raw_path_length:.3f} -> {path_length:.3f} rad")
    apply_rrt_path_to_dm_control(model, rrt_path, video_name="rrt_robot_motion_connect.mp4")
else:
    print("No path found!")
