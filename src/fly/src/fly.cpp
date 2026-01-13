#include "PID_Controller.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream> // 包含此头文件以使用文件流
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <string>
#include <std_msgs/Int32.h>
#include <deque>
#include <array>

std::ofstream outFile_x("example_x.txt");
std::ofstream outFile_y("example_y.txt");
auto start = std::chrono::high_resolution_clock::now();
auto end = std::chrono::high_resolution_clock::now();
double Ts = 0.1;
Eigen::Vector3d current_velocity;

mavros_msgs::State current_state;
enum FLY_MOD {
  READY,
  GET_POINT,
  TAKEOFF,
  UNPRECISE_REACH,
  AUTO_LAND,
  REACH,
  SEARCH,
  DETECT,
  DOWN_DROP,
  RECOVER,
  TAR_LAND,
  MANUAL_LAND,
  END
} state;

struct Point {
  double x = 0;
  double y = 0;
  double z = 0;
} current_position, home, waypoint[10], land, target, fusion_pose, fusion_vel,
    traj_pose, traj_vel, traj_acc, land_point, target_point;

Eigen::Quaterniond q_fcu;

//  圆或H图标
double current_yaw;
double current_roll;
double current_pitch;
double error = 0.3;
double precise_err = 0.1;

// 自定义函数
double error_vel = 0.25;
double tar_height = 0.5;
double fly_height = 0.8;
double search_height = 0.6;
double hover_height = 0.5;
double drop_height = 0.3;
bool is_drop = false;
bool is_match;
bool get_tar;
int drop_count = 0;
int now_goal = 0;
int no_tar = 0;

std_msgs::Bool drop_msg;
std_msgs::Bool tar_ask;

Eigen::Vector3d thrust;
double limit_velocity(double expect_vel, int mode) {
  double limit_vel;
  // mode 决定速度限制为多少
  if (mode == 0) {
    if (fabs(expect_vel) > 0.5)
      limit_vel = 0.4 * expect_vel / fabs(expect_vel);
    else
      limit_vel = expect_vel;
  } else if (mode == 1) {
    if (fabs(expect_vel) > 0.8)
      limit_vel = 0.8 * expect_vel / fabs(expect_vel);
    else
      limit_vel = expect_vel;
  } else if (mode == 2) {
    if (fabs(expect_vel) > 1.2)
      limit_vel = 1.2 * expect_vel / fabs(expect_vel);
    else
      limit_vel = expect_vel;
  }
  return limit_vel;
}

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q) {
  double quat[4];
  quat[0] = q.w();
  quat[1] = q.x();
  quat[2] = q.y();
  quat[3] = q.z();

  Eigen::Vector3d ans;
  ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
  ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
  ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
                 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
  return ans;
} // 四元数转换为欧拉角

//  飞控状态
void state_cb(const mavros_msgs::State::ConstPtr &msg) // 回调函数
{
  current_state = *msg;
}
// 获取姿态
double get_ros_time(ros::Time begin) {
  ros::Time time_now = ros::Time::now();
  double currTimeSec = time_now.sec - begin.sec;
  double currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimeSec + currTimenSec);
}
void pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  q_fcu = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  current_position.x = msg->pose.pose.position.x;
  current_position.y = msg->pose.pose.position.y;
  current_position.z = msg->pose.pose.position.z;
  Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
  // rpy
  current_roll = euler_fcu[0];
  current_pitch = euler_fcu[1];
  current_yaw = euler_fcu[2];
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  current_velocity(0) = msg->twist.linear.x;
  current_velocity(1) = msg->twist.linear.y;
  current_velocity(2) = msg->twist.linear.z;
}
bool get_traj = false;
std::deque<Eigen::VectorXd> trajDeque;
Eigen::VectorXd lastTraj = Eigen::VectorXd::Zero(6);;
Eigen::VectorXd nowTraj = Eigen::VectorXd::Zero(6);;
Eigen::VectorXd tmp1Traj = Eigen::VectorXd::Zero(6);;
Eigen::VectorXd tmp2Traj = Eigen::VectorXd::Zero(6);;
bool initFlag = false;
void traj_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
  traj_pose.x = msg->position.x;
  //outFile_x << "traj_x" << traj_pose.x << std::endl;
  traj_pose.y = msg->position.y;
  //outFile_y << "traj_y" << traj_pose.y << std::endl;
  traj_pose.z = msg->position.z;

  traj_vel.x = msg->velocity.x;
  traj_vel.y = msg->velocity.y;
  traj_vel.z = msg->velocity.z;
  if(!initFlag){
    tmp1Traj << traj_pose.x, traj_pose.y, traj_pose.z, traj_vel.x, traj_vel.y, traj_vel.z;
    trajDeque.push_back(tmp1Traj);  
    nowTraj = trajDeque.back();
    initFlag = true;
  }
  lastTraj = trajDeque.back();
  if(fabs(traj_pose.x - lastTraj(0)) + fabs(traj_pose.y - lastTraj(1)) + fabs(traj_pose.z - lastTraj(2)) > 0.2) {
    tmp1Traj << traj_pose.x, traj_pose.y, traj_pose.z, traj_vel.x, traj_vel.y, traj_vel.z;
    trajDeque.push_back(tmp1Traj);  
  }
  
  // traj_acc.x = msg->acceleration.x;
  // traj_acc.y = msg->acceleration.y;
  // traj_acc.z = msg->acceleration.z;
  get_traj = true;
}
double home_yaw;
bool get_target = false;
void tar_cb(const geometry_msgs::PointConstPtr &msg) {
  target_point.x = msg->x;

  outFile_x << "traj_x" << traj_pose.x << std::endl;
  target_point.y = msg->y;
  outFile_y << "traj_y" << traj_pose.y << std::endl;
  target_point.z = msg->z;
  // traj_acc.x = msg->acceleration.x;
  // traj_acc.y = msg->acceleration.y;
  // traj_acc.z = msg->acceleration.z;
  get_target = true;
}
void land_cb(const geometry_msgs::PointConstPtr &msg) {
  land_point.x = msg->x;
  land_point.y = msg->y;
}
void matchResultCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (drop_count <= 2 && (msg->data || ((4 - now_goal) == (3 - drop_count)))) {
    is_match = true;
  } else {
    is_match = false;
  }
}
void istarResultCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data ) {
    get_tar = true;
  } else {
    get_tar = false;
  }
}
Eigen::Vector3d pos_error = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector3d vel = Eigen::Vector3d(0.0, 0.0, 0.0);
// void cmdCallback(const ros::TimerEvent &e){
//   if(pos_error(0) + pos_error(1) + pos_error(2) < 0.10) {
//     nowTraj = trajDeque.front();
//     trajDeque.pop_front();
//     }
// }

int main(int argc, char **argv) {
  //真实世界
  ros::init(argc, argv, "offb_node"); 
  // 初始化一个名为 "offb_node" 的 ROS节点。argc 和 argv 是命令行参数，用于传递给 ROS。 
  ros::NodeHandle nh; //创建了一个 ROS 节点句柄。
  // Subscribers：
  ros::Subscriber state_sub =nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb); //每当接收到新的消息时,将调用 state_cb，获取飞控状态 
  ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10,pos_cb); // 获取当前位置和姿态作 
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10, vel_cb);// 获取当前速度 
  ros::Subscriber traj_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 10, traj_cb);
  //  Publishers：
  //  用于发布目标点
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>(
      "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::Publisher tar_ask_pub = nh.advertise<std_msgs::Int8>(
    "/target_ask",10);
  ros::Publisher traj_goal =
  nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  // 解锁
  ros::ServiceClient arming_client =
  nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  // 切换飞控模式
  ros::ServiceClient set_mode_client = 
  nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); 
  ros::Subscriber cir_sub = nh.subscribe<geometry_msgs::Point>("/target_position", 10, tar_cb);
  ros::Subscriber land_sub =nh.subscribe<geometry_msgs::Point>("/land_point", 10, land_cb);
  ros::Publisher drop_pub = nh.advertise<std_msgs::Int8>("/drop", 3);
  ros::Subscriber match_sub = nh.subscribe<std_msgs::Bool>("/match_result",10, matchResultCallback);
  ros::Subscriber is_tar_sub = nh.subscribe<std_msgs::Bool>("/is_tar_result",10, istarResultCallback);
   
  // ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.05), cmdCallback);

  // // 虚拟仿真
  // ros::init(argc, argv,
  //           "offb_node"); // 初始化一个名为 "offb_node" 的 ROS 节点。argc 和
  //                         // argv 是命令行参数，用于传递给 ROS。
  // ros::NodeHandle nh; // 创建了一个 ROS 节点句柄。
  // // Subscribers：
  // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
  //     "/iris_0/mavros/state", 1,
  //     state_cb); // 每当接收到新的消息时,将调用 state_cb，获取飞控状态
  // ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>(
  //     "/iris_0/mavros/local_position/odom", 10, pos_cb); // 获取当前位置和姿态作
  // ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(
  //     "/iris_0/mavros/local_position/velocity_local", 10,
  //     vel_cb); // 获取当前速度
  // ros::Subscriber traj_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
  //     "/position_cmd", 10, traj_cb);
  // //  Publishers：
  // //  用于发布目标点
  // ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>(
  //     "/iris_0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
  //     "/iris_0/mavros/setpoint_position/local", 10);
  // ros::Publisher traj_goal =
  //     nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  // ros::Publisher tar_ask_pub = nh.advertise<std_msgs::Int8>(
  //     "/target_ask",10);
  // // 解锁
  // ros::ServiceClient arming_client =
  //     nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");
  // // 切换飞控模式
  // ros::ServiceClient set_mode_client =
  //     nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
  // ros::Subscriber cir_sub =
  //     nh.subscribe<geometry_msgs::Point>("/target_point", 10, tar_cb);
  // ros::Subscriber land_sub =
  //     nh.subscribe<geometry_msgs::Point>("/land_point", 10, land_cb);
  // ros::Publisher drop_pub = nh.advertise<std_msgs::Int8>("/drop", 3);
  // ros::Subscriber match_sub =
  //     nh.subscribe<std_msgs::Bool>("/match_result", 10, matchResultCallback);

  for (int i = 0; i <= 7; i++) {
    nh.param<double>("x" + std::to_string(i), waypoint[i].x,0.0); // 数值类型（如整数、浮点数等）转换为对应的字符串表示
    nh.param<double>("y" + std::to_string(i), waypoint[i].y,0.0);
    ROS_INFO("waypoint_x%d: %.2f", i, waypoint[i].x);
    ROS_INFO("waypoint_y%d: %.2f", i, waypoint[i].y);
  }

  PID::pos_controller_PID pos_controller_pid;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(40.0); // 设置循环频率为20Hz
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }
  // 这个 while 循环会持续运行，直到 ROS 系统关闭或无人机成功连接。
  ROS_INFO("Start To Fly!!!");
  ROS_INFO("fly_height:%.2f", fly_height);

  ros::spinOnce();
  rate.sleep();

  // 发布无人机的飞行目标位置（包括当前高度和即将达到的投放高度）。
  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = 0;
  goal.pose.position.y = 0;
  goal.pose.position.z = 0;
  goal.pose.orientation.w = 1;
  goal.pose.orientation.x = 0;
  goal.pose.orientation.y = 0;
  goal.pose.orientation.z = 0;
  geometry_msgs::Twist pub_vel;
  pub_vel.linear.x = 0;
  pub_vel.linear.y = 0;
  pub_vel.linear.z = 0;

  pub_vel.angular.x = 0;
  pub_vel.angular.y = 0;
  pub_vel.angular.z = 0;
  geometry_msgs::PoseStamped pos_setpoint;
  pos_setpoint.pose.position.x = 0;
  pos_setpoint.pose.position.y = 0;
  pos_setpoint.pose.position.z = 0;
  pos_setpoint.pose.orientation.w = 1;
  pos_setpoint.pose.orientation.x = 0;
  pos_setpoint.pose.orientation.y = 0;
  pos_setpoint.pose.orientation.z = 0;

  local_pos_pub.publish(pos_setpoint);

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::SetMode manual_set_mode;
  manual_set_mode.request.custom_mode = "MANUAL";
  mavros_msgs::SetMode land_set_mode;
  land_set_mode.request.custom_mode = "AUTO.LAND";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  state = READY;
  ros::Time begin_time = ros::Time::now();
  ros::Time last_request = ros::Time::now();
  ros::Time last_request1 = ros::Time::now();
  ros::Time last_request2 = ros::Time::now();
  double last_time = get_ros_time(begin_time);
  double delta_time = 0;
  double cur_time;
  ros::Time start_time = ros::Time::now();
  ros::Time drop_start_time;

  //  send a few setpoints before starting
  //  否则飞控会拒绝接入
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(pos_setpoint);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok()) {

    switch (state) {
    case READY:
      // 解锁
      // ROS_INFO("state READY");
      // 距离上次请求超过 0.5 秒
      if (current_state.mode != "OFFBOARD" &&
          (ros::Time::now() - last_request > ros::Duration(0.3))) {
        // 切换为OFFBOARD
        if (set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent) {
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      } else if (current_state.mode == "OFFBOARD" && current_state.armed) {
        ROS_INFO("Vehicle armed");
        // state = TAKEOFF_PLUS;
        state = TAKEOFF;
        ROS_INFO("state TAKEOFF");
        // 获取xyz和yaw
        // home用于记录最初始的状态
        home.x = current_position.x;
        home.y = current_position.y;
        home.z = current_position.z;
        last_request = ros::Time::now();
        break;
      } else {
        if (!current_state.armed &&
            (ros::Time::now() - last_request >
             ros::Duration(
                 0.3))) // 如果无人机未解锁（!current_state.armed）且从上一次请求解锁到现在的时间超过了0.5秒（ros::Time::now()
                        // - last_request >
                        // ros::Duration(0.5)），则尝试解锁无人机。
        {
          if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
            state = TAKEOFF;
            ROS_INFO("state TAKEOFF");
            // 获取xyz和yaw
            // home用于记录最初始的状态
            home.x = current_position.x;
            home.y = current_position.y;
            home.z = current_position.z;
            home_yaw = current_yaw;
            last_time = get_ros_time(begin_time);
            end = std::chrono::high_resolution_clock::now();
            break;
          }
          last_request = ros::Time::now();
        }
      }
      break;

    case TAKEOFF:
      pos_error(0) = home.x - current_position.x;
      pos_error(1) = home.y - current_position.y;
      pos_error(2) = home.z + fly_height - current_position.z;
      cur_time = get_ros_time(begin_time);
      // start = std::chrono::high_resolution_clock::now();
      // delta_time为启控到当前时间的间隔
      // delta_time = (start - end).count();
      delta_time = cur_time - last_time;
      last_time = cur_time;
      vel = pos_controller_pid.posController(pos_error, current_velocity,
                                             delta_time);
      // ROS_INFO("vel_x:%.3f", vel(0));
      // ROS_INFO("vel_y:%.3f", vel(1));
      // ROS_INFO("vel_z:%.3f", vel(2));
      pub_vel.linear.x = limit_velocity(vel(0), 0);
      pub_vel.linear.y = limit_velocity(vel(1), 0);
      pub_vel.linear.z = limit_velocity(vel(2), 0);
      local_vel_pub.publish(pub_vel);
      if ((fabs(current_position.x - home.x) < error) &&
          (fabs(current_position.y - home.y) < error) &&
          (fabs(current_position.z - home.z - fly_height) < error)) {
        goal.pose.position.x = home.x + waypoint[now_goal].x;
        goal.pose.position.y = home.y + waypoint[now_goal].y;
        goal.pose.position.z = home.z + tar_height;
        traj_goal.publish(goal);
        ROS_INFO("111");
        state = UNPRECISE_REACH;
      }
      // end = std::chrono::high_resolution_clock::now();
      break;

    // go to the first point
    case GET_POINT:
            // ROS_INFO("state GET_POINT");

            if ((!get_traj )&& (ros::Time::now() - last_request > ros::Duration(2.0)))
            { 
                ROS_INFO("no traj!Waiting...");
                goal.pose.position.x = home.x + waypoint[now_goal].x;
                goal.pose.position.y = home.y + waypoint[now_goal].y;
                goal.pose.position.z = home.z + tar_height;
                traj_goal.publish(goal);
                pub_vel.linear.x = 0;
                pub_vel.linear.y = 0;
                pub_vel.linear.z = 0;
                local_vel_pub.publish(pub_vel);
                last_request = ros::Time::now();
                break;
            }
            // ROS_INFO("state GET_POINT");
            cur_time = get_ros_time(begin_time);
            // delta_time为启控到当前时间的间隔
            delta_time = cur_time - last_time;
            last_time = cur_time;
            // if(pos_error(0) + pos_error(1) + pos_error(2) < 0.08) {
            //   nowTraj = trajDeque.front();
            //   trajDeque.pop_front();
            //   pos_error(0) = home.x +  nowTraj[0] - current_position.x;
            //   pos_error(1) = home.y +  nowTraj[1] - current_position.y;
            //   pos_error(2) = home.z +  nowTraj[2] - current_position.z;
            //   local_vel_pub.publish(pub_vel);
            // }
            // pos_error(0) = home.x +  nowTraj(0) - current_position.x;
            // pos_error(1) = home.y +  nowTraj(1) - current_position.y;
            // pos_error(2) = home.z +  nowTraj(2) - current_position.z;
            
            pos_error(0) = home.x +  traj_pose.x - current_position.x;
            pos_error(1) = home.y +  traj_pose.y - current_position.y;
            pos_error(2) = home.z +  traj_pose.z - current_position.z;
            vel = pos_controller_pid.posController(pos_error, current_velocity, delta_time);
            pub_vel.linear.x = limit_velocity(vel(0), 0);
            pub_vel.linear.y = limit_velocity(vel(1), 0);
            pub_vel.linear.z = limit_velocity(vel(2), 0);
            local_vel_pub.publish(pub_vel);
            // ROS_INFO("vel_x:%.3f", vel(0));
            // ROS_INFO("vel_y:%.3f", vel(1));
            // ROS_INFO("vel_z:%.3f", vel(2));
            // 在目标点偏差范围之内
              //ROS_INFO("error:%.3f", fabs(current_position.x - waypoint[now_goal].x - home.x));
              //ROS_INFO("error.y:%.3f", fabs(current_position.y - waypoint[now_goal].y - home.y));
            if ((fabs(current_position.x - waypoint[now_goal].x - home.x) < error) &&
                (fabs(current_position.y - waypoint[now_goal].y - home.y) < error) &&
                (fabs(current_position.z - fly_height - home.z) < error) &&
                current_velocity.norm() < error_vel)
            {  
                ROS_INFO("error.x:%.3f", fabs(current_position.x - waypoint[now_goal].x - home.x));
                ROS_INFO("error.y:%.3f", fabs(current_position.y - waypoint[now_goal].y - home.y));
                ROS_INFO("state UNPRECISE REACH");
                state = UNPRECISE_REACH;
            }
            // else if(ros::Time::now() - last_request1 > ros::Duration(15.0))
            // {
            //     ROS_INFO("too long");
            //     state = UNPRECISE_REACH;
            //     last_request1 = ros::Time::now();
            // }
            get_traj = false;
            break;


    case UNPRECISE_REACH:
      pos_error(0) = home.x + waypoint[now_goal].x - current_position.x;
      pos_error(1) = home.y + waypoint[now_goal].y - current_position.y;
      pos_error(2) = home.z + fly_height - current_position.z;
      cur_time = get_ros_time(begin_time);
      // delta_time为启控到当前时间的间隔
      delta_time = cur_time - last_time;
      last_time = cur_time;
      vel = pos_controller_pid.posController(pos_error, current_velocity,
                                             delta_time);
      pub_vel.linear.x = limit_velocity(vel(0), 0);
      pub_vel.linear.y = limit_velocity(vel(1), 0);
      pub_vel.linear.z = limit_velocity(vel(2), 0);
      local_vel_pub.publish(pub_vel);
      if ((fabs(current_position.x - waypoint[now_goal].x - home.x) <
           precise_err) &&
          (fabs(current_position.y - waypoint[now_goal].y - home.y) <
           precise_err) &&
          ((fly_height + home.z - current_position.z ) <
           precise_err) && 
          (fly_height + home.z - current_position.z ) > 0 &&
          current_velocity.norm() < error_vel) {
        ROS_INFO("PRECISE REACH");
        if (now_goal == 0) {
          ROS_INFO("now_goal:%d", now_goal);
          now_goal++;
          last_request1 = ros::Time::now();
          state = GET_POINT;
          goal.pose.position.x = home.x + waypoint[now_goal].x;
          goal.pose.position.y = home.y + waypoint[now_goal].y;
          goal.pose.position.z = home.z + tar_height;
          traj_goal.publish(goal);
        } 
        else if (now_goal >= 1 && now_goal < 5) {
          if(now_goal == 4){
            fly_height = 0.25;
          }
          ROS_INFO("now_goal:%d", now_goal);
        //   state = SEARCH;
        //   ROS_INFO("STATE SEARCH");
          state = GET_POINT;
          now_goal ++;
          last_request1 = ros::Time::now();}
        else if (now_goal >= 5 && now_goal < 7) {
            state = GET_POINT;
            now_goal ++;
          }
        else {
            ROS_INFO("state TAR_LAND");
            state = AUTO_LAND;
          }
      } else if (current_velocity(0) < error_vel &&
                   current_velocity(1) < error_vel &&
                   current_velocity(2) < error_vel) {
          state = UNPRECISE_REACH;
      }

        break;
      case SEARCH:
        tar_ask.data = now_goal;
        tar_ask_pub.publish(tar_ask);
        // ROS_INFO("STATE SEARCH");
        cur_time = get_ros_time(begin_time);
        // delta_time为启控到当前时间的间隔
        delta_time = cur_time - last_time;
        last_time = cur_time;
        pos_error(0) = home.x + waypoint[now_goal].x - current_position.x;
        pos_error(1) = home.y + waypoint[now_goal].y - current_position.y;
        pos_error(2) = home.z + search_height - current_position.z;

        vel = pos_controller_pid.posController(pos_error, current_velocity,
                                               delta_time);
        pub_vel.linear.x = limit_velocity(vel(0), 0);
        pub_vel.linear.y = limit_velocity(vel(1), 0);
        pub_vel.linear.z = limit_velocity(vel(2), 0);
        local_vel_pub.publish(pub_vel);

        if ((fabs(current_position.x - waypoint[now_goal].x - home.x) <
             error) &&
            (fabs(current_position.y - waypoint[now_goal].y - home.y) <
             error) &&
            (fabs(current_position.z - home.z - search_height) < error)) {
          if (get_target) {
            ROS_INFO(" get tar DETECT");
            // state = RECOVER;  
            state = DETECT;
            last_request2 = ros::Time::now();
            get_target = false;
          } else if (ros::Time::now() - last_request1 > ros::Duration(10.0)) {
            // state = RECOVER;
            ROS_INFO(" no tar DETECT,time over");
            state = RECOVER;
            last_request2 = ros::Time::now();
          }
        }

        break;

      case DETECT:
        // ROS_INFO("DETECT");
        // ROS_INFO("TARGET_PONIT_x %f",target_point.x);
        // ROS_INFO("TARGET_PONIT_y %f",target_point.x);
        cur_time = get_ros_time(begin_time);
        // delta_time为启控到当前时间的间隔
        delta_time = cur_time - last_time;
        last_time = cur_time;
        pos_error(0) = home.x + target_point.x - current_position.x;
        pos_error(1) = home.y + target_point.y - current_position.y;
        pos_error(2) = home.z + search_height - current_position.z;
        ROS_INFO("target_point.x:%.3f",target_point.x);
        ROS_INFO("target_point.y:%.3f",target_point.y);
        vel = pos_controller_pid.posController(pos_error, current_velocity,
                                               delta_time);
        pub_vel.linear.x = limit_velocity(vel(0), 0);
        pub_vel.linear.y = limit_velocity(vel(1), 0);
        pub_vel.linear.z = limit_velocity(vel(2), 0);
        local_vel_pub.publish(pub_vel);
        if ((fabs(current_position.x - target_point.x - home.x) <
             precise_err) &&
            (fabs(current_position.y - target_point.y - home.y) <
             precise_err) &&
            (fabs(current_position.z - search_height - home.z) < precise_err) &&
            current_velocity.norm() < error_vel)

        {
          ROS_INFO("is_match:%d",is_match);
          ROS_INFO("is_drop:%d",is_drop);
          if (now_goal == 1 || now_goal == 3 || now_goal == 4) {
            drop_count++;
            ROS_INFO("Detected,Start to drop");
            drop_msg.data = drop_count;
            state = DOWN_DROP;
          }
          if (ros::Time::now() - last_request2 > ros::Duration(15.0))
          // if((is_drop && ros::Time::now() - drop_start_time >
          // ros::Duration(3.0)))
          {

            ROS_INFO("NO DROP , DROPCOUNT:%d",drop_count);
            is_drop = false;
            state = RECOVER;
            ROS_INFO("DETECT Failed");
            ROS_INFO("RECOVER");
          }
        }

        break;

      case DOWN_DROP:
        ROS_INFO("DOWN_DROP");
        cur_time = get_ros_time(begin_time);
        // delta_time为启控到当前时间的间隔
        delta_time = cur_time - last_time;
        last_time = cur_time;
        pos_error(0) = home.x + target_point.x - current_position.x;
        pos_error(1) = home.y + target_point.y - current_position.y;
        pos_error(2) = home.z + drop_height - current_position.z;

        vel = pos_controller_pid.posController(pos_error, current_velocity,
                                               delta_time);
        pub_vel.linear.x = limit_velocity(vel(0), 0);
        pub_vel.linear.y = limit_velocity(vel(1), 0);
        pub_vel.linear.z = limit_velocity(vel(2), 0);
        local_vel_pub.publish(pub_vel);

        if ((fabs(current_position.x - target_point.x - home.x) <
             precise_err) &&
            (fabs(current_position.y - target_point.y - home.y) <
             precise_err) &&
            (fabs(current_position.z - drop_height - home.z) < precise_err) &&
            current_velocity.norm() < error_vel) {
          if (!is_drop) {
            drop_start_time = ros::Time::now();
          }
          drop_pub.publish(drop_msg);
          ROS_INFO("DROP %d", drop_count);
          is_drop = true;
          if (ros::Time::now() - drop_start_time > ros::Duration(3.0)) {
            is_drop = false;
            state = RECOVER;
          }
        }
        break;

      case RECOVER:
        ROS_INFO("recover");
        ROS_INFO("NOW_GAOL:%d",now_goal);
        tar_ask.data = no_tar;
        tar_ask_pub.publish(tar_ask);
        // ROS_INFO("RECOVER");
        pos_error(0) = home.x + waypoint[now_goal].x - current_position.x;
        pos_error(1) = home.y + waypoint[now_goal].y - current_position.y;
        pos_error(2) = home.z + fly_height - current_position.z;
        cur_time = get_ros_time(begin_time);
        // delta_time为启控到当前时间的间隔
        delta_time = cur_time - last_time;
        last_time = cur_time;
        vel = pos_controller_pid.posController(pos_error, current_velocity,
                                               delta_time);
        pub_vel.linear.x = limit_velocity(vel(0), 0);
        pub_vel.linear.y = limit_velocity(vel(1), 0);
        pub_vel.linear.z = limit_velocity(vel(2), 0);
        local_vel_pub.publish(pub_vel);

        if ((fabs(current_position.x - waypoint[now_goal].x - home.x) <error) &&
            (fabs(current_position.y - waypoint[now_goal].y - home.y) <error) &&
            (fabs(current_position.z - fly_height - home.z) < error) &&
            current_velocity.norm() < error_vel) {
          // ROS_INFO("RECOVER HEIGHT");
          now_goal++;
          state = GET_POINT;
          // state = AUTO_LAND;
          goal.pose.position.x = home.x + waypoint[now_goal].x;
          goal.pose.position.y = home.y + waypoint[now_goal].y;
          goal.pose.position.z = home.z + tar_height;
          traj_goal.publish(goal);
        } else if (current_velocity(0) < error_vel &&
                   current_velocity(1) < error_vel &&
                   current_velocity(2) < error_vel) {
          state = RECOVER;
        }
        break;
      case TAR_LAND:
        cur_time = get_ros_time(begin_time);
        // delta_time为启控到当前时间的间隔
        delta_time = cur_time - last_time;
        last_time = cur_time;

        pos_error(0) = home.x + target_point.x - current_position.x;
        pos_error(1) = home.y + target_point.y - current_position.y;
        pos_error(2) = home.z + search_height - current_position.z;

        vel = pos_controller_pid.posController(pos_error, current_velocity,
                                               delta_time);
        pub_vel.linear.x = limit_velocity(vel(0), 0);
        pub_vel.linear.y = limit_velocity(vel(1), 0);
        pub_vel.linear.z = limit_velocity(vel(2), 0);
        local_vel_pub.publish(pub_vel);
        if ((fabs(current_position.x - target_point.x - home.x) <
             precise_err) &&
            (fabs(current_position.y - target_point.y - home.y) <
             precise_err) &&
            (fabs(current_position.z - search_height - home.z) < precise_err) &&
            current_velocity.norm() < error_vel)

        {
          state = AUTO_LAND;
        }

      case AUTO_LAND:
        // 切auto_land降落

        if (current_state.mode == "OFFBOARD") {
          set_mode_client.call(land_set_mode);
          if (land_set_mode.response.mode_sent) {
            ROS_INFO("auto.land enabled");
            state = MANUAL_LAND;
            last_request = ros::Time::now();
          }
        }
        break;

      case MANUAL_LAND:
        if (current_position.z - home.z < 0.1) {

          if (current_state.mode != "OFFBOARD" &&
              (ros::Time::now() - last_request > ros::Duration(0.3))) {
            if (set_mode_client.call(manual_set_mode) &&
                manual_set_mode.response.mode_sent) {
              ROS_INFO("MANUAL enabled");
              arm_cmd.request.value = false;
            }
            last_request = ros::Time::now();
          } else if (current_state.armed &&
                     (ros::Time::now() - last_request > ros::Duration(0.3))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
              ROS_INFO("Vehicle disarmed");
              state = END;
              break;
            }
          }
        }
        break;
      case END:
        ROS_INFO("MISSION COMPLETE!");
        outFile_x.close();
        outFile_y.close();
        break;
      default:
        break;
      }
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
  }