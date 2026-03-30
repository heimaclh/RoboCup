RoboCup 控制代码
无人机自主飞行与任务调度控制节点 (UAV Autonomous Task Controller)
项目简介
本项目是一个基于 ROS 和 MAVROS 的无人机自主飞行控制节点 (offb_node)。该节点内部集成了一个复杂的状态机，用于实现无人机的自动解锁、Offboard 模式切换、多航点巡航、轨迹跟踪、目标视觉搜索与精确对齐，以及物资投放和自主降落等全流程复合任务。底层依靠自定义的 PID 控制器完成从位置误差到目标速度的解算。

环境与依赖 (Dependencies)
ROS (兼容 Melodic / Noetic)

MAVROS (mavros_msgs)

Eigen3 (用于处理矩阵运算、四元数与欧拉角转换)

自定义控制库: 需要包含 PID_Controller.h 及其实现。

节点接口说明 (Interfaces)
订阅的话题 (Subscribers)
/mavros/state (mavros_msgs::State): 获取飞控当前连接状态、模式及解锁状态。

/mavros/local_position/odom (nav_msgs::Odometry): 获取无人机当前局部位置与姿态（用于提取 RPY 角）。

/mavros/local_position/velocity_local (geometry_msgs::TwistStamped): 获取无人机当前局部线速度。

/position_cmd (quadrotor_msgs::PositionCommand): 获取外部规划器下发的期望轨迹位姿与速度。

/target_position (geometry_msgs::Point): 视觉模块回传的目标物具体位置。

/land_point (geometry_msgs::Point): 获取降落点坐标。

/match_result / /is_tar_result (std_msgs::Bool): 接收视觉模块的目标匹配与确认结果。

发布的话题 (Publishers)
/mavros/setpoint_velocity/cmd_vel_unstamped (geometry_msgs::Twist): 向飞控发送解算后的期望速度指令。

mavros/setpoint_position/local (geometry_msgs::PoseStamped): 向飞控发送期望位置节点（主要用于起飞前保持 OFFBOARD 模式心跳）。

/move_base_simple/goal (geometry_msgs::PoseStamped): 发布当前导航目标点（供可视化或规划节点使用）。

/target_ask (std_msgs::Int8): 触发视觉搜索任务。

/drop (std_msgs::Int8): 触发投放机构进行物资投放。

调用的服务 (Service Clients)
/mavros/cmd/arming: 自动解锁/上锁无人机。

/mavros/set_mode: 切换飞行模式（OFFBOARD, AUTO.LAND, MANUAL）。

参数配置 (Parameters)
节点启动时会从 ROS 参数服务器读取预设航点：

x0 ~ x7: 共 8 个航点的 X 坐标。

y0 ~ y7: 共 8 个航点的 Y 坐标。

关键高度参数配置 (内置变量):

fly_height: 默认巡航高度 (0.8m)

search_height: 搜索目标时的高度 (0.6m)

drop_height: 投放物资时的高度 (0.3m)

核心状态机流程 (State Machine Flow)
本节点采用单层枚举状态机调度任务，主要流程如下：

READY: 等待飞控连接，切换 OFFBOARD 模式并执行解锁。

TAKEOFF: 采用 PID 闭环控制飞向初始起飞高度。

GET_POINT / UNPRECISE_REACH: 接收外部轨迹或飞向预设的各个航点，当位置与速度误差满足阈值时触发状态切换。

SEARCH: 到达特定航点后，降低至搜索高度并发布 /target_ask 请求视觉识别。

DETECT: 接收到目标后，无人机根据目标坐标进行 XY 平面的精确对齐。

DOWN_DROP: 对齐完成后，进一步降低至投放高度，发布 /drop 指令执行投放。

RECOVER: 投放完成后迅速爬升恢复至安全巡航高度，并前往下一个航点。

AUTO_LAND / MANUAL_LAND: 遍历完成所有任务点后，触发飞控自动降落并上锁，任务结束。
