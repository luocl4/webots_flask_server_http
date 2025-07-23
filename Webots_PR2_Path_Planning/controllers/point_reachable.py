from dataclasses import dataclass
import numpy as np
from ik.arm_ik import ArmIk
import os
import time
import pydrake
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    MultibodyPlant,
    AddMultibodyPlantSceneGraph,
    RigidTransform,
    DrakeVisualizer,
    RotationMatrix,
    RollPitchYaw,
    Parser,
    MeshcatVisualizer,
)
from pydrake.geometry import StartMeshcat, Sphere, Rgba


class PointRechable:
    """机器人可达点分析工具类"""

    def __init__(
        self,
        robot_urdf_path="./models/v4_pro_arm/urdf/urdf_arm_optimized.urdf",  # 机器人URDF文件路径
        box_urdf_path="./models/box/box_panels.urdf",  # 箱体URDF文件路径
        box_pos=[0, 0, 0],  # 箱体位置[x,y,z](米)
        box_rpy=[0, 0, 0],  # 箱体姿态[roll,pitch,yaw](弧度)
        box_size=[
            58 / 100,
            38 / 100,
            36 / 100,
        ],  # 箱体尺寸[长,宽,高](米), 要随着URDF文件的改变而改变
        collision="None",  # 碰撞检测类型(None/"Box")
        eef_constraint_type='vector',  # 末端约束类型("vector"/"constraint"/"cost")
        torso_z_free=False,  # 是否允许躯干z轴自由活动
        eef_length=0.175,  # 末端执行器z轴偏移(米)
        torso_xy_free=False,  # 是否允许躯干xy轴自由活动
        ori_vector_tol=5,  # 向量夹角约束容差
    ):
        """初始化可达点分析工具

        Args:
            robot_urdf_path: 机器人URDF文件路径,默认使用v4_pro_arm模型
            box_urdf_path: 箱体URDF文件路径,默认使用box_panels模型
            box_pos: 箱体在世界坐标系中的位置[x,y,z],单位:米
            box_rpy: 箱体在世界坐标系中的姿态[roll,pitch,yaw],单位:弧度
            box_size: 箱体尺寸[长,宽,高],单位:米
            collision: 碰撞检测类型
                - "None": 不进行碰撞检测
                - "Box": 与箱体进行碰撞检测
            eef_constraint_type: 末端执行器约束类型
                - "vector": 使用向量夹角约束(默认)
                - "constraint": 使用姿态约束
                - "cost": 使用姿态代价函数
            torso_z_free: 是否允许躯干z轴自由活动
                - True: 躯干z���在±范围内可自由活动
                - False: 躯干z轴固定(默认)
            eef_length: 末端执行器z轴偏移量,单位:米
            torso_xy_free: 是否允许躯干xy轴自由活动
                - True: 躯干xy轴在±范围内可自由活动
                - False: 躯干xy轴固定(默认)
            ori_vector_tol: 向量夹角约束容差,单位:度
        """
        # 保存机器人URDF文件路径
        self.robot_urdf_path = robot_urdf_path
        # 保存箱体URDF文件路径
        self.box_urdf_path = box_urdf_path
        # 保存箱体位置[x,y,z]
        self.box_pos = box_pos
        # 保存箱体姿态[roll,pitch,yaw]
        self.box_rpy = box_rpy
        # 保存箱体尺寸[长,宽,高]
        self.box_size = box_size
        # 保存碰撞检测类型
        self.collision = collision
        # 保存末端的方向约束类型
        self.constraint_type = eef_constraint_type
        # 保存是否允许躯干z轴自由活动
        self.torso_z_free = torso_z_free
        # 保存是否允许躯干xy轴自由活动
        self.torso_xy_free = torso_xy_free
        # 保存末端执行器z轴偏移
        self.eef_length = eef_length
        # 末端连杆方向约束容差
        self.ori_vector_tol = ori_vector_tol

        # 创建ArmIk实例
        self.arm_ik = ArmIk(
            model_file=robot_urdf_path,
            box_file=box_urdf_path,
            eef_z_bias=eef_length,
            box_pos=box_pos,
            box_rpy=box_rpy,
            collision=collision,
            eef_constraint_type=eef_constraint_type,
            torso_z_free=torso_z_free,
            use_custom_eef=True,
            torso_xy_free=torso_xy_free,
            iterations_limit=300,
            ori_vector_tol=ori_vector_tol,
        )
        # 初始化机器人状态
        self.arm_ik.init_state(0.0, 0.0)
        self.last_q = None

        # 左右手关节角限位，要随着URDF文件的改变而改变
        self.right_joints_bound = np.array(
            [
                [-3.14, 1.57],
                [-2.09, 0.349],
                [-1.57, 1.57],
                [-2.62, 1.57],
                [-1.57, 1.57],
                [-0.7, 1.308],
                [-0.7, 0.7],
            ]
        )
        self.left_joints_bound = np.array(
            [
                [-3.14, 1.57],
                [-0.349, 2.09],
                [-1.57, 1.57],
                [-2.62, 1.57],
                [-1.57, 1.57],
                [-1.308, 0.7],
                [-0.7, 0.7],
            ]
        )

        # 保存结果数据的路径
        self.save_dir = f"./sim_data/{str(self.box_pos)}_{str([round(x, 2) for x in self.box_rpy])}_{self.eef_length}_{self.collision}_{self.constraint_type}_z_free:{self.torso_z_free}_xy_free:{self.torso_xy_free}_ori_tol:{self.ori_vector_tol}"

        # 尝试随机初始化关节角度的次数
        self.random_init_times = 15

    def full_ik_calculate(self, end, q_0=None):
        """计算双臂IK
        Args:
            end: 末端位姿 [左手位置,右手位置,左手RPY,右手RPY,左手朝向,右手朝向]
            q_0: 初始关节角度配置
        Returns:
            tuple: (是否成功求解, 关节角度解)
        """
        if q_0 is None:
            self.__init_q = np.zeros(14 + 7)
            self.__init_q[0] = 1.0
            self.__init_q[10] = -0.3
            self.__init_q[17] = -0.3
            q_0 = self.__init_q

        q_now = self.arm_ik.computeIK(
            q_0, end[0], end[1], end[2], end[3], vectors=np.array(end[4:])
        )

        if q_now is None:
            return False, q_0
        else:
            print("solved q is ", q_now.tolist(), "\n")
            return True, q_now

    def calculate_ik(self, end, state, q_0=None):
        """计算单臂IK
        Args:
            end: 末端位姿 [位置,RPY,朝向]
            state: 选择手臂(0:左手,1:右手)
            q_0: 初始关节角度配置
        Returns:
            tuple: (是否成功求解, 关节角度解)
        """
        end = np.array(end)
        if q_0 is None:
            self.__init_q = np.zeros(14 + 7)
            self.__init_q[0] = 1.0
            self.__init_q[10] = -0.31
            self.__init_q[17] = -0.31
            q_0 = self.__init_q

        # 全0状态下的end
        if state == 0:
            sol_q = self.arm_ik.computeIK_left(
                q_0 if self.last_q is None else self.last_q, end[0], end[2], end[4]
            )
            times = 1
            while sol_q is None and times < self.random_init_times:
                self.random_q = np.zeros(14 + 7)
                self.random_q[0] = 1.0
                self.random_q[7:14] = np.random.uniform(
                    self.left_joints_bound[:, 0], self.left_joints_bound[:, 1]
                )
                sol_q = self.arm_ik.computeIK_left(
                    self.random_q, end[0], end[2], end[4]
                )
                times += 1

        elif state == 1:
            sol_q = self.arm_ik.computeIK_right(
                q_0 if self.last_q is None else self.last_q, end[1], end[3], end[5]
            )
            times = 1
            while sol_q is None and times < self.random_init_times:
                self.random_q = np.zeros(14 + 7)
                self.random_q[0] = 1.0
                self.random_q[14:] = np.random.uniform(
                    self.right_joints_bound[:, 0], self.right_joints_bound[:, 1]
                )
                sol_q = self.arm_ik.computeIK_right(
                    self.random_q, end[1], end[3], end[5]
                )
                times += 1

        if sol_q is not None:
            temp_q = np.zeros(21)
            temp_q[0] = 1.0
            temp_q[7:] = sol_q[7:].copy()
            self.last_q = temp_q
            return True, sol_q
        else:
            return False, None

    def box_point_test(self, right_half=True, both_hand=False):
        """在箱体表面进行可达性采样测试
        Args:
            right_half: 是否只测试右半部分，
            both_hand: 是否同时测试左右手
        保存结果:
            success_ids: 成功点的索引
            success_pos: 成功点的位置
            fail_pose: 失败点的位置
            success_sol_q: 成功点对应的关节角度解
            statistics.txt: 统计信息(成功帧数、总帧数、成功率)
            arm_ik_params.txt: ArmIk实例化参数
        """

        end_origin = [
            np.array([-0.0173, 0.2927, -0.2267 - self.eef_length]),
            np.array([-0.0175, -0.2927, -0.2267 - self.eef_length]),  # 左右手pos
            np.array([0.0, 0, 0.0]),
            np.array([0.0, 0, 0]),  # 左右手 rpy
            np.array([0, 0, 1]),
            np.array([0, 0, 1]),
        ]

        success_ids = []
        success_sol_q = []
        success_pos = []
        fail_pose = []

        count = 0
        s_count = 0

        box_length = int(self.box_size[0] * 100)  # 转换为厘米
        box_width = int(self.box_size[1] * 100)
        box_height = int(self.box_size[2] * 100)

        x_range = [-box_length // 2 + 4, 0 if right_half else box_length // 2 - 4]
        y_range = [-box_width // 2 + 4, box_width // 2 - 4]
        z_range = [0 + 3, box_height + 1]

        orientation = RotationMatrix(
            RollPitchYaw(box_rpy[0], box_rpy[1], box_rpy[2])
        ).matrix()

        P1 = np.array([x_range[0], y_range[0], 0]).dot(orientation.T)
        P2 = np.array([x_range[1], y_range[1], 0]).dot(orientation.T)
        P3 = np.array([x_range[0], y_range[1], 0]).dot(orientation.T)
        V1 = P1 - P2
        V2 = P1 - P3
        norm_vertor = np.cross(V1, V2)
        norm_vertor = norm_vertor / np.linalg.norm(norm_vertor)
        end_origin[4] = norm_vertor
        end_origin[5] = norm_vertor

        for x in range(x_range[0], x_range[1] + 3, 3):
            for y in range(y_range[0], y_range[1] + 3, 3):
                for z in range(z_range[1], z_range[0] - 3, -3):
                    pos = orientation.dot(np.array([x / 100, y / 100, z / 100]))
                    pos += np.array(box_pos)
                    end_origin[1] = pos
                    start_time = time.time()
                    success, sol_q = self.calculate_ik(end=end_origin, state=1)
                    print("time cost:", time.time() - start_time)
                    if success:
                        success_ids.append(count)
                        success_pos.append([x / 100, y / 100, z / 100])
                        success_sol_q.append(sol_q)
                        s_count += 1

                    elif both_hand:
                        end_origin[0] = pos
                        success, sol_q = self.calculate_ik(end=end_origin, state=0)
                        if success:
                            success_ids.append(count)
                            success_pos.append([x / 100, y / 100, z / 100])
                            success_sol_q.append(sol_q)
                            s_count += 1
                        else:
                            print(end_origin)
                            fail_pose.append([x / 100, y / 100, z / 100])

                    else:
                        print(end_origin)
                        fail_pose.append([x / 100, y / 100, z / 100])
                    count += 1
                    print(f"处理第{count}个数据")

        # 保存统计信息到txt文件
        success_rate = s_count / count * 100

        # 创建保存路径
        save_dir = self.save_dir
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # 保存统计信息
        with open(f"{save_dir}/statistics.txt", "w") as f:
            f.write(f"成功帧数: {s_count}\n")
            f.write(f"总帧数: {count}\n")
            f.write(f"成功率: {success_rate:.2f}%\n")

        # 保存ArmIk参数
        arm_ik_params = {
            'model_file': self.robot_urdf_path,
            'box_file': self.box_urdf_path,
            'end_frames_name': ["urdf_arm", "l_hand_pitch", "r_hand_pitch"],
            'constraint_tol': self.arm_ik._ArmIk__constraint_tol,
            'solver_tol': self.arm_ik._ArmIk__solver_tol,
            'ori_cost_weight': self.arm_ik._ArmIk__ori_cost_weight,
            'ori_vector_tol': self.arm_ik._ArmIk__ori_vector_tol,
            'ori_constraint_tol': self.arm_ik._ArmIk__ori_constraint_tol,
            'torso_tol': self.arm_ik._ArmIk__torso_tol,
            'torso_ori_tol': self.arm_ik._ArmIk__torso_ori_tol,
            'iterations_limit': self.arm_ik._ArmIk__iterations_limit,
            'torso_z_bound': self.arm_ik._ArmIk__torso_z_bound,
            'geometry_distance_min': self.arm_ik._ArmIk__geometry_distance_min,
            'geometry_distance_max': self.arm_ik._ArmIk__geometry_distance_max,
            'eef_z_bias': self.eef_length,
            'use_custom_eef': True,
            'box_rpy': box_rpy,
            'box_pos': box_pos,
            'collision': self.collision,
            'eef_constraint_type': self.constraint_type,
            'torso_z_free': self.torso_z_free,
        }

        # 保存ArmIk参数
        with open(f"{save_dir}/arm_ik_params.txt", "w") as f:
            f.write("ArmIk实例化参数:\n")
            for key, value in arm_ik_params.items():
                f.write(f"{key}: {value}\n")

        # 保存结果数据
        np.save(f"{save_dir}/success_ids.npy", np.array(success_ids))
        np.save(f"{save_dir}/success_pos.npy", np.array(success_pos))
        np.save(f"{save_dir}/success_sol_q.npy", np.array(success_sol_q))
        np.save(f"{save_dir}/fail_pose.npy", np.array(fail_pose))

    def visualize_in_drake(self, show_points=False, q_0=None, show_fail=False):
        """使用Drake可视化结果
        Args:
            show_points: 是否显示可达点
        显示:
            - 机器人模型
            - 箱体模型
            - 可达点(蓝色小球)
            - 失败点(红色小球)
        """

        save_dir = self.save_dir
        poses_path = f"{save_dir}/success_pos.npy"
        sol_q_path = f"{save_dir}/success_sol_q.npy"
        fail_pose_path = f"{save_dir}/fail_pose.npy"

        meshcat = StartMeshcat()
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)

        parser = Parser(plant)

        robot = parser.AddModels(self.robot_urdf_path)
        box = parser.AddModels(self.box_urdf_path)

        box_frame = plant.GetFrameByName("world_panel")
        rotation = RollPitchYaw(
            self.box_rpy[0], self.box_rpy[1], self.box_rpy[2]
        ).ToRotationMatrix()  # 绕z轴旋转90度
        translation = np.array(self.box_pos)
        transformer = RigidTransform(rotation, translation)
        plant.WeldFrames(plant.world_frame(), box_frame, transformer)

        plant.Finalize()
        # visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

        if show_points:
            if not os.path.exists(poses_path):
                print("错误:找不到位姿数据文件 {}".format(poses_path))
                print("请先运行 box_point_test.py 生成采样点数据")
                return
            points = np.load(poses_path)
            points = points @ (rotation.matrix()).T + translation
            for i, point in enumerate(points):
                sphere_radius = 0.01  # 球的半径
                meshcat.SetObject(
                    f"point_{i}",
                    Sphere(sphere_radius),
                    rgba=Rgba(r=0.0, g=0.0, b=1.0, a=1.0),
                )  # 红色小球
                meshcat.SetTransform(f"point_{i}", RigidTransform(point))

            # 加载失败点
            if show_fail:
                if os.path.exists(fail_pose_path):
                    fail_pose = np.load(fail_pose_path)
                    if len(fail_pose) > 0:
                        fail_pose = fail_pose @ rotation.matrix().T + np.array(
                            self.box_pos
                        )
                        # 绘制失败点
                    for i, point in enumerate(fail_pose):
                        sphere_radius = 0.01  # 球的半径
                        meshcat.SetObject(
                            f"point_{i}",
                            Sphere(sphere_radius),
                            rgba=Rgba(r=01.0, g=0.0, b=0.0, a=1.0),
                        )  # 红色小球
                        meshcat.SetTransform(f"point_{i}", RigidTransform(point))

        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(diagram_context)
        # visualizer.StartRecording()

        q = np.zeros(21)
        q[0] = 1.0
        q = q if q_0 is None else q_0
        while True:
            diagram.ForcedPublish(diagram_context)
            plant.SetPositions(plant_context, q)
            time.sleep(1)

    def show_joint_state(self, q_list, show_box=True, show_points=False):
        """可视化关节角度序列
        Args:
            q_list: 关节角度序列,None则使用保存的结果
            show_box: 是否显示箱体，
            show_points: 是否显示可达点
        """

        save_dir = self.save_dir
        poses_path = f"{save_dir}/success_pos.npy"
        sol_q_path = f"{save_dir}/success_sol_q.npy"

        if q_list is None:
            if not os.path.exists(sol_q_path):
                print("错误:找不到位姿数据文件 {}".format(poses_path))
                print("请先运行 box_point_test.py 生成采样点数据")
                return
            sol_qs = np.load(sol_q_path)

        meshcat = StartMeshcat()
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(plant)
        rotation = RollPitchYaw(
            self.box_rpy[0], self.box_rpy[1], self.box_rpy[2]
        ).ToRotationMatrix()  # 绕z轴旋转90度
        translation = np.array(self.box_pos)
        transformer = RigidTransform(rotation, translation)
        robot = parser.AddModels(self.robot_urdf_path)

        if show_box:
            box = parser.AddModels(self.box_urdf_path)
            box_frame = plant.GetFrameByName("world_panel")
            plant.WeldFrames(plant.world_frame(), box_frame, transformer)

        if show_points:
            if not os.path.exists(poses_path):
                print("错误:找不到位姿数据文件 {}".format(poses_path))
                print("请先运行 box_point_test.py 生成采样点数据")
                return
            points = np.load(poses_path)
            points = points @ (rotation.matrix()).T + translation
            for i, point in enumerate(points):
                sphere_radius = 0.01  # 球的半径
                meshcat.SetObject(
                    f"point_{i}",
                    Sphere(sphere_radius),
                    rgba=Rgba(r=0.0, g=0.0, b=1.0, a=1.0),
                )  # 红色小球
                meshcat.SetTransform(f"point_{i}", RigidTransform(point))

        plant.Finalize()
        visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(diagram_context)
        visualizer.StartRecording()

        index = 0
        while True:
            diagram.ForcedPublish(diagram_context)
            if q_list is None:
                plant.SetPositions(plant_context, sol_qs[index])
            else:
                plant.SetPositions(plant_context, q_list[index])
            index += 1
            time.sleep(0.1)

    def visualize_in_pyplot(self, show_fail=False):
        """使用Matplotlib绘制可达点分布
        显示:
            - 箱体线框(绿色)
            - 可达点(蓝色)
            - 失败点(红色)
        """
        # 加载数据
        save_dir = self.save_dir
        poses_path = f"{save_dir}/success_pos.npy"
        fail_pose_path = f"{save_dir}/fail_pose.npy"
        if not os.path.exists(poses_path):
            print("错误:找不到位姿数据文件 {}".format(poses_path))
            print("请先运行 box_point_test.py 生成采样点数据")
            return
        points = np.load(poses_path)
        print("可达点数量：", len(points))

        # 坐标变换
        rotation = RollPitchYaw(
            self.box_rpy[0], self.box_rpy[1], self.box_rpy[2]
        ).ToRotationMatrix()
        points = points @ rotation.matrix().T + np.array(self.box_pos)

        # 生成箱体顶点坐标
        corners = self._generate_box_corners()
        corners = corners @ rotation.matrix().T + np.array(self.box_pos)

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # 绘制箱体线框
        self._draw_box_edges(ax, corners)

        # 绘制可达点
        ax.scatter(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            c='b',
            marker='o',
            alpha=0.5,
            label='Reachable Points',
        )

        # 加载失败点
        if show_fail:
            if os.path.exists(fail_pose_path):
                fail_pose = np.load(fail_pose_path)
                if len(fail_pose) > 0:
                    fail_pose = fail_pose @ rotation.matrix().T + np.array(self.box_pos)
                    # 绘制失败点
                    ax.scatter(
                        fail_pose[:, 0],
                        fail_pose[:, 1],
                        fail_pose[:, 2],
                        c='r',
                        marker='o',
                        alpha=0.95,
                        label='Fail Points',
                    )

        # 设置图形属性
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Robot Reachable Points Distribution')
        ax.legend()
        ax.grid(True)

        plt.show()

    def _generate_box_corners(self):
        """生成箱体顶点坐标"""
        l, w, h = self.box_size
        return np.array(
            [
                # 上表面
                [-l / 2, -w / 2, h],
                [-l / 2, w / 2, h],
                [l / 2, -w / 2, h],
                [l / 2, w / 2, h],
                # 下表面
                [-l / 2, -w / 2, 0],
                [-l / 2, w / 2, 0],
                [l / 2, -w / 2, 0],
                [l / 2, w / 2, 0],
            ]
        )

    def _draw_box_edges(self, ax, corners):
        """绘制箱体边框"""
        # 上表面
        for i, j in [(0, 1), (0, 2), (1, 3), (2, 3)]:
            ax.plot(
                [corners[i, 0], corners[j, 0]],
                [corners[i, 1], corners[j, 1]],
                [corners[i, 2], corners[j, 2]],
                color='green',
                linewidth=2,
            )

        # 下表面
        for i, j in [(4, 5), (4, 6), (5, 7), (6, 7)]:
            ax.plot(
                [corners[i, 0], corners[j, 0]],
                [corners[i, 1], corners[j, 1]],
                [corners[i, 2], corners[j, 2]],
                color='green',
                linewidth=2,
            )

        # 竖直边
        for i, j in [(0, 4), (1, 5), (2, 6), (3, 7)]:
            ax.plot(
                [corners[i, 0], corners[j, 0]],
                [corners[i, 1], corners[j, 1]],
                [corners[i, 2], corners[j, 2]],
                color='green',
                linewidth=2,
            )

    def calculate_oris_err(self):
        """计算姿态误差统计
        计算:
            - 目标姿态与实际姿态之间的角度误差
        输出:
            最大值、最小值、平均值、中位值
        """

        err = []
        save_dir = self.save_dir
        poses_path = f"{save_dir}/success_pos.npy"
        sol_q_path = f"{save_dir}/success_sol_q.npy"
        if not os.path.exists(sol_q_path):
            print("错误:找不到位姿数据文件 {}".format(poses_path))
            print("请先运行 box_point_test 生成采样点数据")
            return
        joints = np.load(sol_q_path)
        target_oris = RollPitchYaw(0, self.box_rpy[0], 0).ToRotationMatrix().matrix()
        for joint in joints:
            right_pos, right_rpy = self.arm_ik.right_hand_pose(joint)
            sol_oris = RollPitchYaw(right_rpy).ToRotationMatrix().matrix()
            R_rel = np.dot(target_oris.T, sol_oris)
            trace_R_rel = np.trace(R_rel)
            theta = np.arccos((trace_R_rel - 1) / 2)
            theta_deg = np.degrees(theta)
            err.append(theta_deg)
            print(theta_deg)
        err = np.array(err)
        print("最大值:", np.max(err))
        print("最小值:", np.min(err))
        print("平均值:", np.mean(err))
        print("中位值:", np.median(err))

    def calculate_pos_err(self):
        """计算位置误差统计
        计算:
            - 目标位置与实际位置之间的欧氏距离误差
        输出:
            最大值、最小值、平均值、中位值
        """

        err = []
        save_dir = self.save_dir
        poses_path = f"{save_dir}/success_pos.npy"
        sol_q_path = f"{save_dir}/success_sol_q.npy"

        if not os.path.exists(sol_q_path):
            print("错误:找不到位姿数据文件 {}".format(poses_path))
            print("请先运行 box_point_test 生成采样点数据")
            return
        joints = np.load(sol_q_path)

        if not os.path.exists(poses_path):
            print("错误:找不到位姿数据文件 {}".format(poses_path))
            print("请先运行 box_point_test 生成采样点数据")
            return
        points = np.load(poses_path)
        rotation = RollPitchYaw(
            self.box_rpy[0], self.box_rpy[1], self.box_rpy[2]
        ).ToRotationMatrix()  # 绕z轴旋转90度
        points = points @ (rotation.matrix()).T + np.array(self.box_pos)

        for i in range(len(joints)):

            right_pos, right_rpy = self.arm_ik.right_hand_pose(joints[i])
            target_end = points[i]
            norm_err = np.linalg.norm(np.array(right_pos) - points[i])
            err.append(norm_err)
            print(norm_err)

        err = np.array(err)
        print("最大值:", np.max(err))
        print("最小值:", np.min(err))
        print("平均值:", np.mean(err))
        print("中位值:", np.median(err))

    def calculate_workspace(
        self,
        x_range=[-0.5, 0.5],  # X轴采样范围（米）
        y_range=[-0.5, 0.5],  # Y轴采样范围（米）
        z_range=[-0.3, 0.5],  # Z轴采样范围（米）
        step=0.05,  # 采样步长（米，越小精度越高，计算量越大）
        arm="right",  # 测试的手臂（"right"/"left"）
    ):
        """计算机械臂的可达空间范围

        Args:
            x_range: X轴采样范围 [min, max]
            y_range: Y轴采样范围 [min, max]
            z_range: Z轴采样范围 [min, max]
            step: 采样步长（单位：米）
            arm: 测试的手臂（右手/左手）
        """
        # 初始化存储结果的列表
        reachable_points = []
        unreachable_points = []

        # 生成三维网格采样点
        x_vals = np.arange(x_range[0], x_range[1], step)
        y_vals = np.arange(y_range[0], y_range[1], step)
        z_vals = np.arange(z_range[0], z_range[1], step)

        # 末端姿态默认值（根据需求调整，如垂直向下）
        target_rpy = np.array([0, 0, 0])  # 目标姿态RPY
        target_vector = np.array([0, 0, -1])  # 末端方向向量（向下）

        # 遍历所有采样点
        total = len(x_vals) * len(y_vals) * len(z_vals)
        count = 0
        print(f"开始采样，总点数：{total}，步长：{step}米")

        for x in x_vals:
            for y in y_vals:
                for z in z_vals:
                    count += 1
                    if count % 100 == 0:
                        print(
                            f"已处理 {count}/{total} 点，进度：{count/total*100:.1f}%"
                        )

                    # 构造末端位姿（位置+姿态）
                    target_pos = np.array([x, y, z])
                    end = [
                        target_pos,  # 左手位置（未使用）
                        target_pos,  # 右手位置
                        target_rpy,  # 左手RPY（未使用）
                        target_rpy,  # 右手RPY
                        target_vector,  # 左手方向向量（未使用）
                        target_vector,  # 右手方向向量
                    ]

                    # 测试可达性（右手/左手）
                    state = 1 if arm == "right" else 0
                    success, _ = self.calculate_ik(end=end, state=state)

                    if success:
                        reachable_points.append([x, y, z])
                    else:
                        unreachable_points.append([x, y, z])

        # 保存结果
        save_dir = f"./workspace_{arm}_step{step}"
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        np.save(f"{save_dir}/reachable_points.npy", np.array(reachable_points))
        np.save(f"{save_dir}/unreachable_points.npy", np.array(unreachable_points))

        print(f"计算完成！结果保存至：{save_dir}")
        print(
            f"可达点数量：{len(reachable_points)}，不可达点数量：{len(unreachable_points)}"
        )
        print(f"可达率：{len(reachable_points)/total*100:.2f}%")


if __name__ == "__main__":

    # 倾斜的时候，合适的参数
    # box_pos=[49/100, 0/100, -20/100], box_rpy=[-30/180 * np.pi, 0, 90/180 * np.pi]
    # 水平的时候，合适的参数
    # box_pos=[39/100, 0/100, -33/100], box_rpy=[0, 0, 90/180 * np.pi]

    # 配置参数
    robot_model_path = (
        "../protos/urdf_arm_mix/urdf/urdf_arm_mix.urdf"  # 机器人模型的URDF文件路径
    )
    box_path = "./models/box/box_panels.urdf"  # 盒子模型的URDF文件路径
    box_pos = [35 / 100, 0 / 100, -33 / 100]  # 箱子在仿真环境中的位置，单位为米
    box_rpy = [0, 0, 90 / 180 * np.pi]  # 箱子在仿真环境中的姿态，单位为弧度
    collision_type = "None"  # 碰撞类型为盒子，不考虑碰撞给“None”
    eef_length = 0.265  # 末端执行器的长度，单位为米
    eef_constraint_type = "vector"  # 末端执行器的约束类型为向量
    torso_z_free = False  # 机器人躯干在Z轴方向是否可自由移动
    torso_xy_free = False  # 机器人躯干在XY平面是否可自由移动
    ori_vector_tol = 30  # 方向向量的容差，单位为度

    # 实例化工具类
    points_tool = PointRechable(
        robot_urdf_path=robot_model_path,
        box_urdf_path=box_path,
        box_pos=box_pos,
        box_rpy=box_rpy,
        collision=collision_type,
        eef_length=eef_length,
        eef_constraint_type=eef_constraint_type,
        torso_z_free=torso_z_free,
        torso_xy_free=torso_xy_free,
        ori_vector_tol=ori_vector_tol,
    )

    # 遍历箱子中的点，计算可达性，生成数据
    # points_tool.box_point_test(right_half=True, both_hand=False)

    # 在Drake中可视化结果
    # points_tool.visualize_in_drake(show_points=False) # show_points=False，可以观察箱子的位置

    # 在Matplotlib中可视化结果
    # points_tool.visualize_in_pyplot(show_fail=True)

    # 显示IK解（关节状态）
    points_tool.show_joint_state(None)

    # 计算姿态误差
    # points_tool.calculate_oris_err()

    # 计算位置误差
    # points_tool.calculate_pos_err()
