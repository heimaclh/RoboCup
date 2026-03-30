没问题，一份优秀的 README 不仅是项目的说明书，更是展示工程能力的门面。我为你重新设计了排版，加入了状态徽章（Badges）、清晰的表格、高亮引用以及更有层次的排版，让它在 GitHub/Gitee 上看起来更加专业和现代化。

你可以直接复制以下内容到你的 `README.md` 文件中：

-----

# 🚁 UAV Autonomous Mission Controller

[](https://www.google.com/search?q=)
[](https://www.google.com/search?q=)
[](https://www.google.com/search?q=)
[](https://www.google.com/search?q=)

> **项目简介** \> 本项目是一个基于 ROS 和 MAVROS 的无人机自主飞行与任务调度控制节点 (`offb_node`)。底层依靠自定义的 PID 控制器完成从位置误差到目标速度的解算，顶层通过复杂状态机实现无人机的自动解锁、多航点巡航、目标视觉搜索对齐、物资精准投放及自主降落等全流程复合任务。

-----

## ✨ 核心特性 (Key Features)

  * **🛡️ 自主接管**：自动等待飞控连接、切换 `OFFBOARD` 模式并执行解锁起飞。
  * **📍 多航点与轨迹跟踪**：支持外部轨迹跟随与预设多航点平滑巡航。
  * **👁️ 视觉闭环对接**：集成视觉目标匹配，支持在特定高度（0.6m）搜索并精确对齐目标。
  * **📦 精准投放逻辑**：对齐后自主下降至空投高度（0.3m）并触发投放机构，随后快速爬升自愈。
  * **📊 数据落盘**：运行期间自动记录 X/Y 轴追踪轨迹至本地 `txt` 文件，便于后续控制精度复盘。

-----

## 🗂 节点接口说明 (Interfaces)

### 📥 订阅话题 (Subscribers)

| 话题名称 (Topic) | 消息类型 (Message Type) | 用途描述 (Description) |
| :--- | :--- | :--- |
| `/mavros/state` | `mavros_msgs::State` | 获取飞控当前连接状态、飞行模式及解锁状态 |
| `/mavros/local_position/odom` | `nav_msgs::Odometry` | 获取无人机当前局部位置与姿态（用于提取 RPY 角） |
| `/mavros/local_position/velocity_local` | `geometry_msgs::TwistStamped` | 获取无人机当前局部线速度反馈 |
| `/position_cmd` | `quadrotor_msgs::PositionCommand`| 获取外部规划器下发的期望轨迹位姿与速度 |
| `/target_position` | `geometry_msgs::Point` | 视觉模块回传的目标物具体局部坐标 |
| `/match_result` | `std_msgs::Bool` | 接收视觉模块的目标匹配与确认结果 |

### 📤 发布话题 (Publishers)

| 话题名称 (Topic) | 消息类型 (Message Type) | 用途描述 (Description) |
| :--- | :--- | :--- |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs::Twist` | 向飞控发送解算后的期望速度指令 |
| `/mavros/setpoint_position/local` | `geometry_msgs::PoseStamped` | 发送期望位置（主要用于起飞前保持 OFFBOARD 心跳） |
| `/move_base_simple/goal` | `geometry_msgs::PoseStamped` | 发布当前导航目标点（供可视化或规划节点使用） |
| `/target_ask` | `std_msgs::Int8` | 触发底层视觉搜索任务 |
| `/drop` | `std_msgs::Int8` | 触发物理投放机构进行物资空投 |

### 🛠️ 调用的服务 (Service Clients)

  * **解锁/上锁**: `/mavros/cmd/arming` (`mavros_msgs::CommandBool`)
  * **模式切换**: `/mavros/set_mode` (`mavros_msgs::SetMode` - 支持 `OFFBOARD`, `AUTO.LAND`, `MANUAL`)

-----

## ⚙️ 参数配置 (Parameters)

节点启动时会从 ROS 参数服务器读取以下预设航点与高度参数：

| 参数类别 | 变量名 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| **预设航点** | `x0` \~ `x7` <br> `y0` \~ `y7` | `0.0` | 共 8 个预设任务航点的 X/Y 坐标 |
| **高度控制** | `fly_height` | `0.8 m` | 默认安全巡航高度 |
| | `search_height` | `0.6 m` | 执行视觉搜索任务时的下降高度 |
| | `drop_height` | `0.3 m` | 触发物资投放时的极限贴地高度 |

-----

## 🔄 状态机流转图 (State Machine Flow)

节点采用枚举状态机调度宏观任务，核心流转顺序如下：

1.  🟢 **READY** -\> 自动切换 `OFFBOARD` 模式并解锁。
2.  🛫 **TAKEOFF** -\> PID 闭环控制飞向初始起飞高度。
3.  🚀 **GET\_POINT / UNPRECISE\_REACH** -\> 巡航至预设航点或跟踪规划轨迹。
4.  🔍 **SEARCH** -\> 降高至 `search_height`，发布识别请求。
5.  🎯 **DETECT** -\> 获取目标坐标，在 XY 平面进行高精度速度闭环对齐。
6.  🪂 **DOWN\_DROP** -\> 下降至 `drop_height` 执行空投任务。
7.  📈 **RECOVER** -\> 投放完成，迅速拉升恢复至巡航高度并切入下一航点。
8.  🛬 **AUTO\_LAND** -\> 任务列表清空，触发飞控自动降落并上锁。

