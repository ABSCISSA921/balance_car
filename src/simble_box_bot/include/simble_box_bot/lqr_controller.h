#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_buffer.h>
#include <Eigen/Dense> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>

namespace simble_box_bot_ns {

class LQRBalanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    LQRBalanceController() = default;
    ~LQRBalanceController() override = default;

    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

private:
    hardware_interface::JointHandle left_joint_, right_joint_;
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);


    // 订阅者
    ros::Subscriber imu_sub_;
    ros::Subscriber cmd_vel_sub_;
    realtime_tools::RealtimeBuffer<sensor_msgs::Imu> imu_buffer_;

    control_toolbox::Pid yaw_pid_;

    double cmd_linear_x_ = 0.0;
    double cmd_yaw_rate_ = 0.0;

    double current_pos_ = 0.0; // 当前位移
    double current_vel_ = 0.0; // 当前速度
    double current_angular_vel_ = 0.0; // 当前绕z角速度
    double current_pitch_ = 0.0; // 当前角度
    double current_pitch_dot_ = 0.0; // 当前pitch角速度

    double target_pitch_ = 0.0;
    double target_pos_ = 0.0;
    double target_linear_vel_ = 0.0;
    double target_angular_vel_ = 0.0;
    double target_pitch_dot_ = 0.0;

    double wheel_radius_ = 0.06;      // 轮子半径

    double last_effort_ = 0.0; // 上一次控制输出
    double k1, k2, k3, k4; // LQR 控制器增益

};

} // namespace simble_box_bot_ns