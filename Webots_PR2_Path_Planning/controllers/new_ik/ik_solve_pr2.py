import sys
import os
import time
import math
import numpy as np
import logging

# 创建日志目录
if not os.path.exists("logs"):
    os.makedirs("logs")

# 配置日志记录
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("logs/arm_ik.log"),  # 输出到文件
        logging.StreamHandler(),  # 也可以输出到控制台
    ],
)
logger = logging.getLogger(__name__)

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    PiecewisePolynomial,
    FixedOffsetFrame,
    RigidTransform,
    InverseKinematics,
    Solve,
    RotationMatrix,
    RollPitchYaw,
    SnoptSolver,
    InitializeAutoDiff,
    JacobianWrtVariable,
    ExtractGradient,
    ExtractValue,
    ComPositionConstraint,
    CentroidalMomentumConstraint,
    StartMeshcat,
    MeshcatVisualizer,
    Cylinder,
    Sphere,
    AngleAxis,
)

# 从geometry模块导入Color类（针对旧版本Drake）
from pydrake.geometry import Rgba as Color
import numpy as np
from scipy.spatial import distance
import time


class ArmIk:
    """机器人双臂运动学控制类

    用于控制机器人双臂和躯干的运动,支持:
    - IK求解(单臂/双臂)
    - 碰撞检测和避障
    - 自定义末端执行器
    - 多种末端约束类型
    - 躯干自由度控制
    """

    def __init__(
        self,
        model_file="../../../Webots_PR2_Path_Planning/protos/pr2/pr2.urdf",
        box_file="./models/box/box_panels.urdf",
        end_frames_name=[
            "base_link",
            "l_gripper_palm_link",
            "r_gripper_palm_link",
        ],
        constraint_tol=9e-3,
        solver_tol=1.0e-3,
        ori_cost_weight=10,
        ori_vector_tol=30,
        ori_constraint_tol=0.15,
        torso_tol=1.0e-6,
        torso_ori_tol=1.0e-6,
        iterations_limit=300,
        torso_z_bound=0.3,
        torso_xy_bound=0.1,
        geometry_distance_min=0.005,
        geometry_distance_max=100,
        eef_offset=[0.18, 0, 0],
        use_custom_eef=True,
        box_rpy=[0, 0, 0],
        box_pos=[0, 0, 0],
        collision="None",
        eef_constraint_type="constraint",
        torso_z_free=False,
        torso_xy_free=False,
        torso_yaw_free=False,
        visualize=False,
    ):
        """初始化机器人控制器"""

        # 创建多体系统和场景图
        builder = DiagramBuilder()
        self.__plant, self.scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
        parser = Parser(self.__plant)

        # 加载机器人和箱体模型
        (robot,) = parser.AddModels(model_file)

        if collision == "Box":
            (box,) = parser.AddModels(box_file)
            self._load_box(box_file, box_pos, box_rpy)

        # 设置自定义末端执行器（在Finalize之前）
        self.use_custom_eef = use_custom_eef
        if use_custom_eef:
            self._setup_custom_eef(end_frames_name, eef_offset)

        # 完成模型构建（锁定模型）
        self.__plant.Finalize()

        # dof = self.__plant.num_positions()
        # logger.info(f"机器人总自由度: {dof}")
        # if dof != 40:
        #     logger.warning(f"检测到自由度不匹配: 代码中假设40，实际{dof}")

        # 初始化Meshcat可视化
        self.visualize = visualize
        if visualize:
            self.meshcat = StartMeshcat()
            MeshcatVisualizer.AddToBuilder(builder, self.scene_graph, self.meshcat)
            self.meshcat.Delete()  # 清空之前的内容
            self.meshcat.SetProperty("/Background", "visible", True)
            self.meshcat.SetProperty("/Background", "color", [240, 240, 240, 255])

        # 构建系统和上下文（在Finalize之后）
        self.__diagram = builder.Build()
        self.__diagram_context = self.__diagram.CreateDefaultContext()
        self.__plant_context = self.__plant.GetMyContextFromRoot(self.__diagram_context)

        # 保存基本参数
        self.__constraint_tol = constraint_tol
        self.__solver_tol = solver_tol
        self.__ori_cost_weight = ori_cost_weight
        self.__ori_vector_tol = ori_vector_tol
        self.__ori_constraint_tol = ori_constraint_tol
        self.__torso_tol = torso_tol
        self.__torso_ori_tol = torso_ori_tol
        self.__iterations_limit = iterations_limit
        self.__torso_z_bound = torso_z_bound
        self.__torso_xy_bound = torso_xy_bound
        self.__geometry_distance_min = geometry_distance_min
        self.__geometry_distance_max = geometry_distance_max
        self.__collision = collision
        self.__eef_constraint_type = eef_constraint_type
        self.__torso_z_free = torso_z_free
        self.__torso_xy_free = torso_xy_free
        self.__torso_yaw_free = torso_yaw_free

        # 初始化碰撞检测
        if collision != "None":
            self._setup_collision_detection()

        # 获取所有需要控制的坐标系
        self.__frames = [self.__plant.GetFrameByName(name) for name in end_frames_name]
        self.__end_eef_names = ["frame_eef_left", "frame_eef_right"]

        # 注意：只有在use_custom_eef为True时，这些框架才存在
        if use_custom_eef:
            self.__end_eef_frames = [
                self.__plant.GetFrameByName(name) for name in self.__end_eef_names
            ]
        else:
            self.__end_eef_frames = self.__frames[1:]  # 使用默认末端关节

        # 初始化状态
        self.init_state(0, 0)

    def _load_box(self, box_file, box_pos, box_rpy):
        """加载箱体模型"""
        box_frame = self.__plant.GetFrameByName("world_panel")
        rotation = RollPitchYaw(box_rpy[0], box_rpy[1], box_rpy[2]).ToRotationMatrix()
        self.__plant.WeldFrames(
            self.__plant.world_frame(), box_frame, RigidTransform(rotation, box_pos)
        )

    # def _setup_custom_eef(self, end_frames_name, eef_z_bias):
    #     """设置自定义末端执行器"""
    #     self.eef_frame_name_list = ["frame_eef_left", "frame_eef_right"]
    #     for i, name in enumerate(end_frames_name[1:]):
    #         eef_frame = self.__plant.GetFrameByName(name)
    #         p = np.array([0, 0, -eef_z_bias])
    #         self.__plant.AddFrame(
    #             FixedOffsetFrame(
    #                 self.eef_frame_name_list[i], eef_frame, RigidTransform(p)
    #             )
    #         )

    def _setup_custom_eef(self, end_frames_name, eef_offset):
        """设置自定义末端执行器框架"""
        self.eef_frame_name_list = ["frame_eef_left", "frame_eef_right"]

        # 为左右手创建自定义框架
        for i, name in enumerate(end_frames_name[1:]):  # 跳过第一个框架（躯干）
            try:
                # 获取基础框架
                base_frame = self.__plant.GetFrameByName(name)

                # 创建偏移变换 - 在Z轴方向偏移
                p = np.array(eef_offset)
                transform = RigidTransform(p)

                # 添加固定偏移框架
                self.__plant.AddFrame(
                    FixedOffsetFrame(self.eef_frame_name_list[i], base_frame, transform)
                )
                logger.info(
                    f"成功创建自定义末端执行器框架: {self.eef_frame_name_list[i]}"
                )

            except Exception as e:
                logger.error(f"创建自定义末端执行器框架失败: {e}")
                # 回退到使用基础框架
                self.__plant.AddFrame(
                    FixedOffsetFrame(
                        self.eef_frame_name_list[i],
                        base_frame,
                        RigidTransform.Identity(),
                    )
                )
                logger.warning(f"使用基础框架作为回退: {name}")

    def _setup_collision_detection(self):
        """初始化碰撞检测"""
        # 获取碰撞检测所需的几何体ID
        self.__robot_left_geometries_ids = {
            "l_arm_yaw": None,
            "l_foream": None,
            "l_hand_yaw": None,
            "l_hand_roll": None,
            "zarm_l7_end_effector": None,
            "l_camera": None,
        }
        self.__robot_right_geometries_ids = {
            "r_arm_yaw": None,
            "r_foream": None,
            "r_hand_yaw": None,
            "r_hand_roll": None,
            "zarm_r7_end_effector": None,
            "r_camera": None,
        }
        self.__box_geometries_ids = {
            "base_panel": None,
            "back_panel": None,
            "front_panel": None,
            "left_panel": None,
            "right_panel": None,
        }

        self.__base_geometries_ids = {"base_link": None}

        # 获取几何体ID
        scene_context = self.scene_graph.GetMyContextFromRoot(self.__diagram_context)
        query_object = self.scene_graph.get_query_output_port().Eval(scene_context)
        inspector = query_object.inspector()

        for geometry_id in inspector.GetAllGeometryIds():
            frame_id = inspector.GetFrameId(geometry_id)
            body = self.__plant.GetBodyFromFrameId(frame_id)
            if body.name() in self.__robot_left_geometries_ids:
                self.__robot_left_geometries_ids[body.name()] = geometry_id
            if body.name() in self.__robot_right_geometries_ids:
                self.__robot_right_geometries_ids[body.name()] = geometry_id
            if body.name() in self.__box_geometries_ids:
                self.__box_geometries_ids[body.name()] = geometry_id
            if body.name() in self.__base_geometries_ids:
                self.__base_geometries_ids[body.name()] = geometry_id

    def _create_ik_solver(self):
        """创建IK求解器"""
        ik = InverseKinematics(
            plant=self.__plant,
            plant_context=self.__plant_context,
            with_joint_limits=True,
        )
        snopt = SnoptSolver().solver_id()
        ik.prog().SetSolverOption(
            snopt, "Major Optimality Tolerance", self.__solver_tol
        )
        ik.prog().SetSolverOption(
            snopt, "Major Iterations Limit", self.__iterations_limit
        )
        return ik

    def _add_collision_constraints(
        self,
        ik,
        arm_type,
    ):
        """添加碰撞约束
        Args:
            ik: IK求解器
            arm_type: 手臂类型("left"/"right")
        """
        # 加载点云
        # 给点云和机器人几何体添加碰撞约束

        if arm_type == "left":
            robot_geometries = self.__robot_left_geometries_ids  # 左手
            orther_hand_geometries = self.__robot_right_geometries_ids
        else:
            robot_geometries = self.__robot_right_geometries_ids  # 右手
            orther_hand_geometries = self.__robot_left_geometries_ids

        for robot_geometry_name, robot_id in robot_geometries.items():
            for box_id in self.__box_geometries_ids.values():

                # 添加手臂与箱体的碰撞约束
                ik.AddDistanceConstraint(
                    (robot_id, box_id),
                    self.__geometry_distance_min,
                    self.__geometry_distance_max,
                )

                # 添加箱体和基座之间的碰撞约束
                if self.__torso_xy_free:
                    ik.AddDistanceConstraint(
                        (box_id, self.__base_geometries_ids["base_link"]),
                        self.__geometry_distance_min,
                        self.__geometry_distance_max,
                    )

            # 添加与另外一只手的碰撞
            for orther_hand_id in orther_hand_geometries.values():
                ik.AddDistanceConstraint(
                    (robot_id, orther_hand_id),
                    self.__geometry_distance_min,
                    self.__geometry_distance_max,
                )

            # 添加与基座的碰撞约束
            ik.AddDistanceConstraint(
                (robot_id, self.__base_geometries_ids["base_link"]),
                self.__geometry_distance_min,
                self.__geometry_distance_max,
            )

    def _add_torso_constraints(self, ik, pose):
        """添加躯干约束

        Args:
            ik: IK求解器
            pose: 躯干目标位姿 [[roll,pitch,yaw], [x,y,z]]
        """

        # 获取躯干坐标系
        # torso_frame = self.__frames[0]
        torso_frame = self.__plant.GetFrameByName("base_link")

        # 添加位置约束
        if not self.__torso_xy_free and not self.__torso_z_free:
            # 全约束
            ik.AddPositionConstraint(
                frameB=torso_frame,
                p_BQ=np.zeros(3),
                frameA=self.__plant.world_frame(),
                p_AQ_lower=np.array(pose[1]) - self.__torso_tol,
                p_AQ_upper=np.array(pose[1]) + self.__torso_tol,
            )

        elif self.__torso_z_free and not self.__torso_xy_free:
            # Z轴约束
            ik.AddPositionConstraint(
                torso_frame,
                [0, 0, 0],
                self.__plant.world_frame(),
                p_AQ_lower=pose[1] - np.array([0, 0, self.__torso_z_bound]),
                p_AQ_upper=pose[1] + np.array([0, 0, self.__torso_z_bound]),
            )
        elif self.__torso_xy_free and not self.__torso_z_free:
            ik.AddPositionConstraint(
                torso_frame,
                [0, 0, 0],
                self.__plant.world_frame(),
                p_AQ_lower=pose[1]
                - np.array([self.__torso_xy_bound, self.__torso_xy_bound, 0]),
                p_AQ_upper=pose[1]
                + np.array([self.__torso_xy_bound, self.__torso_xy_bound, 0]),
            )

        else:
            ik.AddPositionConstraint(
                torso_frame,
                [0, 0, 0],
                self.__plant.world_frame(),
                p_AQ_lower=pose[1]
                - np.array(
                    [
                        self.__torso_xy_bound,
                        self.__torso_xy_bound - 0,
                        self.__torso_z_bound,
                    ]
                ),
                p_AQ_upper=pose[1]
                + np.array(
                    [
                        self.__torso_xy_bound - 0.05,
                        self.__torso_xy_bound - 0,
                        self.__torso_z_bound,
                    ]
                ),
            )

        # 添加姿态约束
        if not self.__torso_yaw_free:
            # 完整姿态约束
            # logger.debug("添加躯干完整姿态约束")
            ik.AddOrientationConstraint(
                torso_frame,
                RotationMatrix(),
                self.__plant.world_frame(),
                RotationMatrix(RollPitchYaw(pose[0])),
                self.__torso_ori_tol,
            )
        else:
            # 只约束Roll和Pitch
            # logger.debug("添加躯干Roll和Pitch约束")
            R_WB = RotationMatrix(RollPitchYaw(pose[0]))
            ik.AddAngleBetweenVectorsConstraint(
                torso_frame,
                [0, 0, 1],  # 躯干z轴
                self.__plant.world_frame(),
                R_WB.col(2),  # 世界坐标系z轴
                0,  # 最小角度
                self.__torso_ori_tol,  # 最大角度
            )

    def _add_end_effector_constraints(self, ik, pose, frame_idx, vectors, arm_type):
        """添加末端执行器约束
        Args:
            ik: IK求解器
            pose: 末端位姿 [RPY, 位置]
            frame_idx: 坐标系索引
            vectors: 朝向向量
            arm_type: 手臂类型("left"/"right")
        """
        frame = (
            self.__frames[frame_idx]
            if not self.use_custom_eef
            else self.__end_eef_frames[0 if arm_type == "left" else 1]
        )

        # 添加姿态约束
        if self.__eef_constraint_type == "vector":
            ik.AddAngleBetweenVectorsConstraint(
                frameA=self.__plant.world_frame(),
                na_A=vectors,
                frameB=frame,
                nb_B=np.array([0, 0, 1]),
                angle_lower=np.radians(0),
                angle_upper=np.radians(self.__ori_vector_tol),
            )
        elif self.__eef_constraint_type == "constraint":
            ik.AddOrientationConstraint(
                self.__plant.world_frame(),
                RotationMatrix(RollPitchYaw(pose[0])),
                frame,
                RotationMatrix(RollPitchYaw(0, 0, 0)),
                self.__ori_constraint_tol,
            )
        elif self.__eef_constraint_type == "cost":
            ik.AddOrientationCost(
                self.__plant.world_frame(),
                RotationMatrix(RollPitchYaw(pose[0])),
                frame,
                RotationMatrix(RollPitchYaw(0, 0, 0)),
                self.__ori_cost_weight,
            )

        # # 添加位置约束
        # ik.AddPositionConstraint(
        #     frameB=frame,
        #     p_BQ=np.zeros(3),
        #     frameA=self.__plant.world_frame(),
        #     p_AQ_lower=np.array(pose[1]) - self.__constraint_tol,
        #     p_AQ_upper=np.array(pose[1]) + self.__constraint_tol,
        # )

        # 换成软 cost
        p_BQ = np.zeros(3)
        p_AQ_desired = np.array(pose[1])  # 期望位置
        weight = 1.0  # 权重，可按需要放大/缩小

        ik.AddPositionCost(
            self.__plant.world_frame(),  # frameA
            np.array(pose[1]),  # p_AP
            frame,  # frameB
            np.zeros(3),  # p_BQ
            np.eye(3) * weight,  # C
        )

    def computeIK(
        self,
        q0,
        l_hand_pose,
        r_hand_pose,
        l_hand_RPY=None,
        r_hand_RPY=None,
        vectors=np.array([[0, 0, 1], [0, 0, 1]]),
    ):
        """计算双臂IK
        Args:
            q0: 初始关节角度
            l_hand_pose: 左手目标位置
            r_hand_pose: 右手目标位置
            l_hand_RPY: 左手目标姿态(Roll-Pitch-Yaw)
            r_hand_RPY: 右手目标姿态
            vectors: 手臂朝向向量
        Returns:
            numpy.ndarray: 求解成功返回关节角度，失败返回None
        """
        # print("computeIK",q0,l_hand_pose,r_hand_pose,l_hand_RPY,r_hand_RPY,vectors)
        start_time = time.time()  # 记录开始时间
        # logger.debug("开始求解双臂IK")
        # 创建IK求解器
        ik = self._create_ik_solver()

        # 设置躯干位姿
        torsoR = [0.0, 0.0, 0.0]
        r = [0.0, 0.0, q0[6]]
        pose_list = [[torsoR, r], [l_hand_RPY, l_hand_pose], [r_hand_RPY, r_hand_pose]]

        # 添加碰撞约束
        if self.__collision != "None":
            self._add_collision_constraints(ik, "left")
            self._add_collision_constraints(ik, "right")

        # # 添加躯干约束
        self._add_torso_constraints(ik, pose_list[0])
        # 添加躯干约束（使用base_link框架）
        # torso_frame = self.__plant.GetFrameByName("base_link")
        # self._add_torso_constraints(ik, pose_list[0], torso_frame)

        # 添加左手约束
        self._add_end_effector_constraints(
            ik=ik, pose=pose_list[1], frame_idx=1, vectors=vectors[0], arm_type="left"
        )

        # 添加右手约束
        self._add_end_effector_constraints(
            ik=ik, pose=pose_list[2], frame_idx=2, vectors=vectors[1], arm_type="right"
        )

        # 添加左手关节固定约束
        # for i in range(7, 14):  # 左手关节索引范围是7-13
        #     ik.prog().AddBoundingBoxConstraint(
        #         q0[i],  # 下界：当前关节角度
        #         q0[i],  # 上界：当前关节角度
        #         ik.q()[i]  # 变量：对应关节
        #     )

        # 固定高度不升降
        torso_lift_joint_index = 19
        ik.prog().AddBoundingBoxConstraint(
            # q0[torso_lift_joint_index],  # 下界：当前关节角度
            # q0[torso_lift_joint_index],  # 上界：当前关节角度
            0.2,
            0.2,
            ik.q()[torso_lift_joint_index],  # 变量：对应关节
        )

        # 求解IK
        result = Solve(ik.prog(), q0)
        # print("result:",result)
        # print("result.GetSolution()",result.GetSolution())
        # print("result.is_success()",result.is_success())

        if result.is_success():
            logger.info(
                f"IK computation successful in {time.time() - start_time:.4f} seconds"
            )
            return result.GetSolution()
        else:
            logger.warning("IK computation failed")
            return None

    def computeIK_left(
        self, q0, l_hand_pose, l_hand_RPY=None, l_norm=np.array([0, 0, 1])
    ):
        """求解左臂IK
        Args:
            q0: 初始关节角度
            l_hand_pose: 左手目标位置
            l_hand_RPY: 左手目标姿态(Roll-Pitch-Yaw)
            l_norm: 手臂朝向向量
        Returns:
            numpy.ndarray: 求解成功返回关节角度，失败返回None
        """
        # 创建IK求解器
        ik = self._create_ik_solver()

        # 设置躯干位姿
        torsoR = [0.0, 0.0, 0.0]
        r = [0.0, 0.0, q0[6]]
        pose_list = [
            [torsoR, r],
            [l_hand_RPY, l_hand_pose],
            l_norm,
        ]

        # 添加碰撞约束
        if self.__collision != "None":
            self._add_collision_constraints(ik, "left")

        # 添加躯干约束
        self._add_torso_constraints(ik, pose_list[0])

        # 添加左手约束
        self._add_end_effector_constraints(
            ik=ik, pose=pose_list[1], frame_idx=1, vectors=pose_list[2], arm_type="left"
        )

        # 添加右手关节固定约束
        for i in range(33, 40):  # 右手关节索引范围是33-40
            ik.prog().AddBoundingBoxConstraint(
                q0[i] - 1e-10,  # 下界：当前关节角度
                q0[i] + 1e-10,  # 上界：当前关节角度
                ik.q()[i],  # 变量：对应关节
            )

        # 求解IK
        result = Solve(ik.prog(), q0)
        return result.GetSolution() if result.is_success() else None

    def computeIK_right(
        self, q0, r_hand_pose, r_hand_RPY=None, r_norm=np.array([0, 0, 1])
    ):
        """求解右臂IK
        Args:
            q0: 初始关节角度
            r_hand_pose: 右手目标位置
            r_hand_RPY: 右手目标姿态
            r_norm: 手臂朝向向量
        Returns:
            numpy.ndarray: 求解成功返回关节角度，失败返回None
        """
        # 创建IK求解器
        ik = self._create_ik_solver()
        # 设置躯干位姿
        torsoR = [0.0, 0.0, 0.0]
        r = [0.0, 0.0, q0[6]]
        pose_list = [
            [torsoR, r],
            [r_hand_RPY, r_hand_pose],
            r_norm,
        ]

        # 添加碰撞约束
        if self.__collision != "None":
            self._add_collision_constraints(ik, "right")

        # 添加躯干约束
        self._add_torso_constraints(ik, pose_list[0])

        # 添加右手约束
        self._add_end_effector_constraints(
            ik=ik,
            pose=pose_list[1],
            frame_idx=2,
            vectors=pose_list[2],
            arm_type="right",
        )

        # 添加左手关节固定约束
        for i in range(26, 33):  # 左手关节索引范围是7-13
            ik.prog().AddBoundingBoxConstraint(
                q0[i] - 1e-10,  # 下界：当前关节角度
                q0[i] + 1e-10,  # 上界：当前关节角度
                ik.q()[i],  # 变量：对应关节
            )

        # 求解IK
        result = Solve(ik.prog(), q0)
        return result.GetSolution() if result.is_success() else None

    def left_hand_jacobian(self, q):
        """计算左手雅可比矩阵
        Args:
            q: 当前关节角度配置
        Returns:
            numpy.ndarray: 左手末端执行器的空间速度雅可比矩阵
        """
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context,
            JacobianWrtVariable.kV,
            self.__frames[1] if not self.use_custom_eef else self.__end_eef_frames[0],
            [0, 0, 0],
            self.__plant.world_frame(),
            self.__plant.world_frame(),
        )
        return J_hand_in_world[:, 6:13]  # 返回左臂7个关节对应的雅可比矩阵

    def right_hand_jacobian(self, q):
        """计算右手雅可比矩阵
        Args:
            q: 当前关节角度配置
        Returns:
            numpy.ndarray: 右手末端执行器的空间速度雅可比矩阵
        """
        self.__plant.SetPositions(self.__plant_context, q)
        J_hand_in_world = self.__plant.CalcJacobianSpatialVelocity(
            self.__plant_context,
            JacobianWrtVariable.kV,
            self.__frames[2] if not self.use_custom_eef else self.__end_eef_frames[1],
            [0, 0, 0],
            self.__plant.world_frame(),
            self.__plant.world_frame(),
        )
        return J_hand_in_world[:, -7:]  # 返回右臂7个关节对应的雅可比矩阵

    def left_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)

        # 打印左臂关节角度
        left_joints = q[7:14]  # 左臂关节索引7-13
        print(f"左臂关节角度: {left_joints}")

        # 打印从基座到末端的变换链
        frame_names = [
            "base_link",  # 基座
            "l_shoulder_pan_link",  # 左肩关节
            "l_shoulder_lift_link",  # 左肩抬升关节
            "l_upper_arm_roll_link",  # 左上臂旋转关节
            "l_elbow_flex_link",  # 左肘弯曲关节
            "l_forearm_roll_link",  # 左前臂旋转关节
            "l_wrist_flex_link",  # 左腕弯曲关节
            "l_wrist_roll_link",  # 左腕旋转关节
            "l_gripper_palm_link",  # 左手掌框架
        ]

        for name in frame_names:
            try:
                frame = self.__plant.GetFrameByName(name)
                pose = frame.CalcPoseInWorld(self.__plant_context)
                print(f"{name} 位置: {pose.translation()}")
            except Exception as e:
                print(f"警告: 未找到框架 {name}: {e}")

        # 获取末端执行器位姿
        frame = (
            self.__frames[1] if not self.use_custom_eef else self.__end_eef_frames[0]
        )
        pose = frame.CalcPoseInWorld(self.__plant_context)
        return (pose.translation(), pose.rotation().ToRollPitchYaw().vector())

    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)

        # 打印右臂关节角度
        right_joints = q[14:21]  # 右臂关节索引14-20
        print(f"右臂关节角度: {right_joints}")

        # 打印从基座到末端的变换链
        frame_names = [
            "base_link",  # 基座
            "r_shoulder_pan_link",  # 右肩关节
            "r_shoulder_lift_link",  # 右肩抬升关节
            "r_upper_arm_roll_link",  # 右上臂旋转关节
            "r_elbow_flex_link",  # 右肘弯曲关节
            "r_forearm_roll_link",  # 右前臂旋转关节
            "r_wrist_flex_link",  # 右腕弯曲关节
            "r_wrist_roll_link",  # 右腕旋转关节
            "r_gripper_palm_link",  # 右手掌框架
        ]

        for name in frame_names:
            try:
                frame = self.__plant.GetFrameByName(name)
                pose = frame.CalcPoseInWorld(self.__plant_context)
                print(f"{name} 位置: {pose.translation()}")
            except Exception as e:
                print(f"警告: 未找到框架 {name}: {e}")

        # 获取末端执行器位姿
        frame = (
            self.__frames[2] if not self.use_custom_eef else self.__end_eef_frames[1]
        )
        pose = frame.CalcPoseInWorld(self.__plant_context)
        return (pose.translation(), pose.rotation().ToRollPitchYaw().vector())

    def right_camera_pose(self, q):
        """获取右手相机在世界坐标系下的位姿
        Args:
            q: 当前关节角度配置
        Returns:
            tuple: (位置向量, RPY角度向量)
        """
        self.__plant.SetPositions(self.__plant_context, q)
        frame = self.__plant.GetFrameByName("r_camera")
        pose = frame.CalcPoseInWorld(self.__plant_context)
        return (pose.translation(), pose.rotation().ToRollPitchYaw().vector())

    def left_camera_pose(self, q):
        """获取左手相机在世界坐标系下的位姿
        Args:
            q: 当前关节角度配置
        Returns:
            tuple: (位置向量, RPY角度向量)
        """
        self.__plant.SetPositions(self.__plant_context, q)
        frame = self.__plant.GetFrameByName("l_camera")
        pose = frame.CalcPoseInWorld(self.__plant_context)
        return (pose.translation(), pose.rotation().ToRollPitchYaw().vector())

    def init_state(self, torso_yaw, torso_z):
        """初始化机器人状态
        Args:
            torso_yaw: 躯干偏航角(弧度)
            torso_z: 躯干z轴位置(米)
        """
        self.__torso_yaw_rad = torso_yaw
        self.__init_q = np.zeros(45)  # 机器人所有关节数为40
        self.__init_q[0] = 1.0

    def get_arm_length(self):
        """计算机器人双臂的当前长度
        Returns:
            tuple: (左臂长度, 右臂长度)
        """
        # 计算左臂长度
        frame_left = (
            self.__frames[1] if not self.use_custom_eef else self.__end_eef_frames[0]
        )
        X_eef_left = frame_left.CalcPoseInWorld(self.__plant_context)
        shoulder_frame_left = self.__plant.GetFrameByName("l_arm_pitch")
        X_shoulder_left = shoulder_frame_left.CalcPoseInWorld(self.__plant_context)
        dis = X_shoulder_left.translation() - X_eef_left.translation()
        length_left = np.linalg.norm(dis)

        # 计算右臂长度
        frame_right = (
            self.__frames[2] if not self.use_custom_eef else self.__end_eef_frames[1]
        )
        X_eef_right = frame_right.CalcPoseInWorld(self.__plant_context)
        shoulder_frame_right = self.__plant.GetFrameByName("r_arm_pitch")
        X_shoulder_right = shoulder_frame_right.CalcPoseInWorld(self.__plant_context)
        dis = X_shoulder_right.translation() - X_eef_right.translation()
        length_right = np.linalg.norm(dis)

        return length_left, length_right

    def get_two_frame_dis(self, frame_a_name, frame_b_name):
        """计算两个坐标系之间的距离
        Args:
            frame_a_name: 坐标系A的名称
            frame_b_name: 坐标系B的名称
        Returns:
            float: 两个坐标系原点之间的欧氏距离
        """
        frame_a = self.__plant.GetFrameByName(frame_a_name)
        X_a = frame_a.CalcPoseInWorld(self.__plant_context)
        frame_b = self.__plant.GetFrameByName(frame_b_name)
        X_b = frame_b.CalcPoseInWorld(self.__plant_context)
        dis = X_a.translation() - X_b.translation()
        return np.linalg.norm(dis)

    def get_last_solution(self):
        """获取上一次求解的结果
        Returns:
            numpy.ndarray: 上一次求解成功的关节角度配置
        """
        return self.__last_solution

    def set_init_q(self, q):
        """设置初始关节角度配置
        Args:
            q: 关节角度配置
        """
        self.__init_q = q

    def get_init_q(self):
        """获取初始关节角度配置
        Returns:
            numpy.ndarray: 初始关节角度配置
        """
        return self.__init_q

    def forward_kinematics(self, q, arm_type="both"):
        """计算机器人正向运动学（FK），获取末端执行器位姿

        正向运动学根据给定的关节角度，计算末端执行器在世界坐标系中的位置和姿态

        Args:
            q (numpy.ndarray): 机器人关节角度配置（40维向量）
            arm_type (str): 计算的手臂类型，可选"left"、"right"或"both"

        Returns:
            dict: 包含末端执行器位姿的字典
                - 当arm_type为"left"时，返回{"left_pos": 位置, "left_rpy": 姿态}
                - 当arm_type为"right"时，返回{"right_pos": 位置, "right_rpy": 姿态}
                - 当arm_type为"both"时，返回{"left_pos": 左位置, "left_rpy": 左姿态,
                                              "right_pos": 右位置, "right_rpy": 右姿态}
        """
        # 设置机器人关节角度
        self.__plant.SetPositions(self.__plant_context, q)

        result = {}

        # 计算左臂FK
        if arm_type in ["left", "both"]:
            left_pos, left_rpy = self._calculate_arm_pose("left")
            result["left_pos"] = left_pos
            result["left_rpy"] = left_rpy

        # 计算右臂FK
        if arm_type in ["right", "both"]:
            right_pos, right_rpy = self._calculate_arm_pose("right")
            result["right_pos"] = right_pos
            result["right_rpy"] = right_rpy

        return result

    def calculate_pose_error(self, target_pose, actual_pose):
        """计算位姿误差（位置误差和姿态误差）

        Args:
            target_pose (dict): 目标位姿字典，包含"position"和"rpy"
            actual_pose (dict): 实际位姿字典，包含"position"和"rpy"

        Returns:
            dict: 误差字典，包含"position_error"和"orientation_error"
        """
        # 计算位置误差（欧氏距离）
        position_error = np.linalg.norm(
            np.array(target_pose["position"]) - np.array(actual_pose["position"])
        )

        # 计算姿态误差（RPY角度差的欧氏距离）
        orientation_error = np.linalg.norm(
            np.array(target_pose["rpy"]) - np.array(actual_pose["rpy"])
        )

        return {
            "position_error": position_error,
            "orientation_error": orientation_error,
        }

    def print_actuator_positions(self, q, target_pose=None, arm_type="both"):
        """打印执行器实际位置，可选择与目标位姿比较计算误差

        Args:
            q (numpy.ndarray): 关节角度配置
            target_pose (dict, optional): 目标位姿字典
            arm_type (str): 手臂类型，可选"left"、"right"或"both"
        """
        # 计算实际位姿
        actual_pose = self.forward_kinematics(q, arm_type)

        print(f"\n=== 执行器实际位置 ===")
        if arm_type in ["left", "both"]:
            print(
                f"左臂位置: x={actual_pose['left_pos'][0]:.6f}m, y={actual_pose['left_pos'][1]:.6f}m, z={actual_pose['left_pos'][2]:.6f}m"
            )
            print(
                f"左臂姿态: roll={actual_pose['left_rpy'][0]:.6f}rad, pitch={actual_pose['left_rpy'][1]:.6f}rad, yaw={actual_pose['left_rpy'][2]:.6f}rad"
            )

            # 如果提供了目标位姿，计算误差
            if target_pose and "left_pos" in target_pose:
                error = self.calculate_pose_error(
                    {
                        "position": target_pose["left_pos"],
                        "rpy": target_pose["left_rpy"],
                    },
                    {
                        "position": actual_pose["left_pos"],
                        "rpy": actual_pose["left_rpy"],
                    },
                )
                print(f"左臂位置误差: {error['position_error']:.6f}m")
                print(f"左臂姿态误差: {error['orientation_error']:.6f}rad")

        if arm_type in ["right", "both"]:
            print(
                f"右臂位置: x={actual_pose['right_pos'][0]:.6f}m, y={actual_pose['right_pos'][1]:.6f}m, z={actual_pose['right_pos'][2]:.6f}m"
            )
            print(
                f"右臂姿态: roll={actual_pose['right_rpy'][0]:.6f}rad, pitch={actual_pose['right_rpy'][1]:.6f}rad, yaw={actual_pose['right_rpy'][2]:.6f}rad"
            )

            # 如果提供了目标位姿，计算误差
            if target_pose and "right_pos" in target_pose:
                error = self.calculate_pose_error(
                    {
                        "position": target_pose["right_pos"],
                        "rpy": target_pose["right_rpy"],
                    },
                    {
                        "position": actual_pose["right_pos"],
                        "rpy": actual_pose["right_rpy"],
                    },
                )
                print(f"右臂位置误差: {error['position_error']:.6f}m")
                print(f"右臂姿态误差: {error['orientation_error']:.6f}rad")

        return actual_pose

    def _calculate_arm_pose(self, arm_type):
        """计算单臂末端执行器位姿

        Args:
            arm_type (str): 手臂类型，"left"或"right"

        Returns:
            tuple: (位置向量, RPY角度向量)
        """
        if arm_type == "left":
            frame = (
                self.__frames[1]
                if not self.use_custom_eef
                else self.__end_eef_frames[0]
            )
        else:  # "right"
            frame = (
                self.__frames[2]
                if not self.use_custom_eef
                else self.__end_eef_frames[1]
            )

        # 计算末端执行器在世界坐标系中的位姿
        pose = frame.CalcPoseInWorld(self.__plant_context)
        position = pose.translation()
        rpy = pose.rotation().ToRollPitchYaw().vector()

        return position, rpy

    def forward_kinematics_with_jacobian(self, q, arm_type="both"):
        """计算正向运动学并返回雅可比矩阵

        Args:
            q (numpy.ndarray): 机器人关节角度配置
            arm_type (str): 计算的手臂类型，可选"left"、"right"或"both"

        Returns:
            dict: 包含位姿和雅可比矩阵的字典
                - 当arm_type为"left"时，返回{"pose": 位姿, "jacobian": 雅可比矩阵}
                - 当arm_type为"right"时，返回{"pose": 位姿, "jacobian": 雅可比矩阵}
                - 当arm_type为"both"时，返回{"left_pose": 左位姿, "left_jacobian": 左雅可比,
                                              "right_pose": 右位姿, "right_jacobian": 右雅可比}
        """
        result = {}

        # 计算左臂FK和雅可比
        if arm_type in ["left", "both"]:
            left_pos, left_rpy = self._calculate_arm_pose("left")
            left_jacobian = self.left_hand_jacobian(q)
            result["left_pose"] = {"position": left_pos, "rpy": left_rpy}
            result["left_jacobian"] = left_jacobian

        # 计算右臂FK和雅可比
        if arm_type in ["right", "both"]:
            right_pos, right_rpy = self._calculate_arm_pose("right")
            right_jacobian = self.right_hand_jacobian(q)
            result["right_pose"] = {"position": right_pos, "rpy": right_rpy}
            result["right_jacobian"] = right_jacobian

        return result

    def get_arm_trajectory(self, q_path, time_interval=0.1):
        """计算手臂轨迹的正向运动学位姿序列

        Args:
            q_path (numpy.ndarray): 关节角度轨迹，形状为[轨迹点数, 40]
            time_interval (float): 时间间隔（秒）

        Returns:
            dict: 包含左右臂轨迹的字典
                - "time": 时间序列
                - "left_pos": 左臂位置轨迹
                - "left_rpy": 左臂姿态轨迹
                - "right_pos": 右臂位置轨迹
                - "right_rpy": 右臂姿态轨迹
        """
        num_points = q_path.shape[0]
        time_sequence = np.arange(0, num_points * time_interval, time_interval)

        left_pos_trajectory = np.zeros((num_points, 3))
        left_rpy_trajectory = np.zeros((num_points, 3))
        right_pos_trajectory = np.zeros((num_points, 3))
        right_rpy_trajectory = np.zeros((num_points, 3))

        for i in range(num_points):
            pose = self.forward_kinematics(q_path[i])
            left_pos_trajectory[i] = pose["left_pos"]
            left_rpy_trajectory[i] = pose["left_rpy"]
            right_pos_trajectory[i] = pose["right_pos"]
            right_rpy_trajectory[i] = pose["right_rpy"]

        return {
            "time": time_sequence,
            "left_pos": left_pos_trajectory,
            "left_rpy": left_rpy_trajectory,
            "right_pos": right_pos_trajectory,
            "right_rpy": right_rpy_trajectory,
        }

    def visualize_pose(self, q, target_pose=None, duration=0.1):
        """在Meshcat中可视化机器人位姿、目标位置和实际位置"""
        if not self.visualize:
            return

        # 更新机器人模型姿态
        self.__plant.SetPositions(self.__plant_context, q)
        self.__diagram.ForcedPublish(self.__diagram_context)
        time.sleep(0.1)  # 等待可视化更新

        # 可视化目标位置（左手深蓝色，右手深橙色）
        if target_pose:
            if "left_pos" in target_pose:
                self.meshcat.SetObject(
                    "/left_target", Sphere(0.02), Color(0, 0, 1, 0.8)
                )
                self.meshcat.SetTransform(
                    "/left_target", RigidTransform(target_pose["left_pos"])
                )

            if "right_pos" in target_pose:
                self.meshcat.SetObject(
                    "/right_target", Sphere(0.02), Color(1, 0.5, 0, 0.8)
                )
                self.meshcat.SetTransform(
                    "/right_target", RigidTransform(target_pose["right_pos"])
                )

        # 可视化实际位置（左手red，右手浅橙色）
        actual_pose = self.forward_kinematics(q)
        if "left_pos" in actual_pose:
            self.meshcat.SetObject("/left_actual", Sphere(0.02), Color(1, 0.0, 0, 0.8))
            self.meshcat.SetTransform(
                "/left_actual", RigidTransform(actual_pose["left_pos"])
            )

        if "right_pos" in actual_pose:
            self.meshcat.SetObject("/right_actual", Sphere(0.02), Color(1, 0.8, 0, 0.8))
            self.meshcat.SetTransform(
                "/right_actual", RigidTransform(actual_pose["right_pos"])
            )

        # # 绘制目标与实际位置的连线（左手蓝色，右手橙色）
        # if target_pose and actual_pose:
        #     if "left_pos" in target_pose and "left_pos" in actual_pose:
        #         self._draw_line(
        #             target_pose["left_pos"], actual_pose["left_pos"],
        #             color=Color(0, 0, 1, 0.5), name="/left_error"
        #         )

        #     if "right_pos" in target_pose and "right_pos" in actual_pose:
        #         self._draw_line(
        #             target_pose["right_pos"], actual_pose["right_pos"],
        #             color=Color(1, 0.5, 0, 0.5), name="/right_error"
        #         )

        time.sleep(duration)  # 保持可视化

    def _draw_line(self, p1, p2, color=Color(1, 0, 0, 1), name="/line"):
        """在Meshcat中绘制两点之间的连线"""
        if not self.visualize:
            return

        # 计算线段中点和方向
        mid_point = (p1 + p2) / 2
        direction = p2 - p1
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return

        direction = direction / length  # 单位向量

        # 创建线段几何体（使用圆柱体模拟）
        radius = 0.005
        self.meshcat.SetObject(name, Cylinder(radius, length), color)

        # 设置线段姿态（沿方向向量）
        R = RotationMatrix.MakeFromOneVector(direction, 0)
        self.meshcat.SetTransform(name, RigidTransform(R, mid_point))

    def print_joint_info(self):
        """打印所有关节信息及自由度"""
        from pydrake.multibody.tree import JointIndex

        print("\n=== 机器人关节信息 ===")
        print(f"总自由度: {self.__plant.num_positions()}")

        # 遍历所有关节
        for joint_idx in range(self.__plant.num_joints()):
            joint = self.__plant.get_joint(JointIndex(joint_idx))
            if joint.num_positions() > 0:  # 忽略固定关节
                print(f"关节 {joint.name()}: {joint.num_positions()} 个位置变量")

        # 修正末端执行器框架检查
        print("\n=== 末端执行器相关关节 ===")
        for frame_name in self.__end_eef_names:
            try:
                frame = self.__plant.GetFrameByName(frame_name)
                body = frame.body()

                # 修正：通过BodyFrame获取关节
                if body.is_floating():
                    print(f"{frame_name} 关联到浮动体: {body.name()}")
                else:
                    # 尝试获取关节（如果有关节）
                    try:
                        joint = self.__plant.GetJointByBodyFrameId(body.frame_id())
                        print(
                            f"{frame_name} 关联关节: {joint.name()}, DOF: {joint.num_positions()}"
                        )
                    except:
                        print(f"{frame_name} 关联到固定连接，无活动关节")

            except Exception as e:
                print(f"未找到框架: {frame_name}, 错误: {e}")

    def calculate_eef_pose(self, yao_joint, joint_angles, goal_arm="both", joint_num=7):
        """
        根据指定手臂的关节角度计算末端执行器位姿（修改为6个关节）

        Args:
            joint_angles (np.ndarray): 关节角度数组
                - 若goal_arm为"left"或"right"：长度为6（对应单臂6个关节）
                - 若goal_arm为"both"：长度为12（前6个为左手，后6个为右手）
            goal_arm (str): 计算目标，可选"left"（左手）、"right"（右手）、"both"（双手）

        Returns:
            dict: 包含末端执行器位姿的字典
                - 若goal_arm为"left"：{"left_position": [x,y,z], "left_rpy": [r,p,y]}
                - 若goal_arm为"right"：{"right_position": [x,y,z], "right_rpy": [r,p,y]}
                - 若goal_arm为"both"：包含上述所有键的字典
        """
        # 验证目标手臂参数
        if goal_arm not in ["left", "right", "both"]:
            raise ValueError(f"goal_arm必须为'left'/'right'/'both'，实际为{goal_arm}")
        # 验证输入关节角度长度（修改为6个关节）
        if goal_arm in ["left", "right"] and len(joint_angles) != joint_num:
            raise ValueError(
                f"单臂关节角度应为{joint_num}个，实际提供了{len(joint_angles)}个"
            )
        if goal_arm == "both" and len(joint_angles) != 2 * joint_num:
            raise ValueError(
                f"双臂关节角度应为{2*joint_num}个，实际提供了{len(joint_angles)}个"
            )

        # 创建完整关节配置
        q = self.get_init_q().copy()

        # 定义关节索引映射（根据URDF模型调整）
        joint_indices = {
            "left": 34,  # 左手关节在完整配置中的起始索引
            "right": 23,  # 右手关节在完整配置中的起始索引
        }
        q[19] = yao_joint

        # 更新目标手臂的关节角度（修改为6个）
        if goal_arm in ["left", "both"]:
            # 提取左手关节角度（若为both，前6个为左手）
            left_angles = (
                joint_angles[:joint_num] if goal_arm == "both" else joint_angles
            )
            q[joint_indices["left"] : joint_indices["left"] + joint_num] = left_angles

        if goal_arm in ["right", "both"]:
            # 提取右手关节角度（若为both，后6个为右手）
            right_angles = (
                joint_angles[joint_num:] if goal_arm == "both" else joint_angles
            )
            q[joint_indices["right"] : joint_indices["right"] + joint_num] = (
                right_angles
            )

        # 设置关节角度到模型中
        self.__plant.SetPositions(self.__plant_context, q)

        # 存储结果的字典
        result = {}

        # 计算左手末端执行器位姿
        if goal_arm in ["left", "both"]:
            left_frame = (
                self.__end_eef_frames[0] if self.use_custom_eef else self.__frames[1]
            )
            left_pose = left_frame.CalcPoseInWorld(self.__plant_context)
            result["left_position"] = left_pose.translation()
            result["left_rpy"] = left_pose.rotation().ToRollPitchYaw().vector()

        # 计算右手末端执行器位姿
        if goal_arm in ["right", "both"]:
            right_frame = (
                self.__end_eef_frames[1] if self.use_custom_eef else self.__frames[2]
            )
            right_pose = right_frame.CalcPoseInWorld(self.__plant_context)
            result["right_position"] = right_pose.translation()
            result["right_rpy"] = right_pose.rotation().ToRollPitchYaw().vector()

        return result


# 主函数部分保持不变...
if __name__ == "__main__":
    # 创建机器人运动学控制器实例，启用可视化
    arm_ik = ArmIk(visualize=True)

    arm_ik.print_joint_info()

    # 获取初始关节角度
    q0 = arm_ik.get_init_q()

    # 打印初始位姿
    print("q0", q0)
    l_pose = arm_ik.left_hand_pose(q0)
    r_pose = arm_ik.right_hand_pose(q0)
    print(f"初始位姿 - 左臂: 位置={l_pose[0]}, 姿态={l_pose[1]}")
    print(f"初始位姿 - 右臂: 位置={r_pose[0]}, 姿态={r_pose[1]}")

    # 定义目标位置
    l_hand_pose = np.array([0.6949, 0.2145, 0.7405])  # [x, y, z] 单位m
    r_hand_pose = np.array([0.7188, -0.0152, 1.0619])

    # 目标位姿和约束向量
    end_origin = [
        l_hand_pose,
        r_hand_pose,  # 左右手位置
        np.array([0, 0, 0]),  # 左手RPY
        np.array([0, 0, 0]),  # 右手RPY
        np.array([0, 0, 1]),  # 左手朝向向量
        np.array([0, 0, 1]),  # 右手朝向向量
    ]

    # 求解双臂IK
    start_time = time.time()
    sol_q = arm_ik.computeIK(
        q0,
        end_origin[0],
        end_origin[1],
        end_origin[2],
        end_origin[3],
        vectors=np.array([[0, 0, 1], [0, 0, 1]]),
    )
    # sol_q = arm_ik.computeIK_left(q0,end_origin[0],end_origin[2])
    # sol_q = arm_ik.computeIK_right(q0,end_origin[1],end_origin[3],end_origin[5])
    print(f"IK求解时间: {time.time() - start_time:.4f}秒")
    # 检查求解是否成功
    if sol_q is not None:
        print("IK求解成功，关节角度配置:")
        print(sol_q)

        # 定义目标位姿字典
        target_pose = {
            "left_pos": l_hand_pose,
            "left_rpy": end_origin[2],
            "right_pos": r_hand_pose,
            "right_rpy": end_origin[3],
        }

        # 打印执行器实际位置并计算误差
        print("\n=== 执行器实际位置与目标误差 ===")
        actual_pose = arm_ik.print_actuator_positions(sol_q, target_pose)

        # 可视化目标位置和实际位置
        print("\n=== 正在打开Meshcat可视化界面 ===")
        print("请在浏览器中访问 http://127.0.0.1:7000 查看可视化结果")
        arm_ik.visualize_pose(sol_q, target_pose)

        # 计算并打印正向运动学结果（验证）
        fk_result = arm_ik.forward_kinematics(sol_q)
        print("\n=== 正向运动学验证 ===")
        print(f"左臂位置: {fk_result['left_pos']}, 姿态: {fk_result['left_rpy']}")
        print(f"右臂位置: {fk_result['right_pos']}, 姿态: {fk_result['right_rpy']}")
        time.sleep(10000)
    else:
        print("IK求解失败")
