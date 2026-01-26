#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h> 
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <realtime_tools/realtime_buffer.h> 
#include <dynamic_reconfigure/server.h>
#include <simble_box_bot/BalanceParamConfig.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>

namespace simble_box_bot_ns {

class BalanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    BalanceController() = default;
    ~BalanceController() override = default;

    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

private:
 
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    typedef simble_box_bot::BalanceParamConfig ConfigT;
    std::shared_ptr<dynamic_reconfigure::Server<ConfigT>> dyn_server_;
    
    void dynamicParamCallback(ConfigT& config, uint32_t level);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    hardware_interface::JointHandle left_joint_, right_joint_;

    ros::Subscriber imu_sub_;
    ros::Subscriber cmd_vel_sub_;

    control_toolbox::Pid pid_controller_;
    control_toolbox::Pid velocity_pid_;
    control_toolbox::Pid yaw_pid_;

    realtime_tools::RealtimeBuffer<sensor_msgs::Imu> imu_buffer_;
    ros::Time last_publish_time_;

    double wheel_radius_{0.06}; // 默认值，实际从参数服务器读取
    double track_width_{0.175}; // 默认值

    // 新增：速度环 PID 参数（简单 P 控制通常够用，也可以用完整 PID）
    double vel_kp_{0.0}; 
    double turn_kp_{0.0};

    // 新增：指令缓存
    double cmd_linear_x_{0.0};
    double cmd_angular_z_{0.0};
    
    // 新增：用于状态切换检测的标志位
    bool is_balancing_{false};
};

} // namespace simble_box_bot_ns