#include "simble_box_bot/balance_controller.h"
#include <tf/tf.h>

namespace simble_box_bot_ns {

bool BalanceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {

    std::string left_joint, right_joint;
    if (!nh.getParam("left_wheel_name", left_joint) || !nh.getParam("right_wheel_name", right_joint)) {
        ROS_ERROR("Could not find joint name params");
        return false;
    }

    left_joint_ = hw->getHandle(left_joint);
    right_joint_ = hw->getHandle(right_joint);

    nh.param("wheel_radius", wheel_radius_, 0.06); 
    nh.param("track_width", track_width_, 0.175);
    
    double y_p, y_i, y_d;
    nh.param("yaw_kp", y_p, 2.0); 
    nh.param("yaw_ki", y_i, 0.0);
    nh.param("yaw_kd", y_d, 0.1);
    yaw_pid_.initPid(y_p, y_i, y_d, 3.0, -3.0);
    
    double v_p, v_i, v_d;
    nh.param("vel_kp", v_p, 0.3); 
    nh.param("vel_ki", v_i, 0.1);
    nh.param("vel_kd", v_d, 0.0);  
    velocity_pid_.initPid(v_p, v_i, v_d, 0.5, -0.5); 

    pid_controller_.initPid(20.0, 0.1, 1.0, 5.0, -5.0);

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu_data", 1, &BalanceController::imuCallback, this);
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &BalanceController::cmdVelCallback, this);
    
    dyn_server_ = std::make_shared<dynamic_reconfigure::Server<ConfigT>>(nh);
    dyn_server_->setCallback(
        boost::bind(&BalanceController::dynamicParamCallback, this, _1, _2)
    );

    return true;
}

void BalanceController::dynamicParamCallback(ConfigT& config, uint32_t level) {
    pid_controller_.setGains(config.kp, config.ki, config.kd, 10.0, 0.0);
   
}

void BalanceController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_linear_x_ = msg->linear.x;
    cmd_angular_z_ = msg->angular.z;
}

void BalanceController::starting(const ros::Time& time) {
    left_joint_.setCommand(0.0);
    right_joint_.setCommand(0.0);
    pid_controller_.reset();
    velocity_pid_.reset();
    yaw_pid_.reset();
}

void BalanceController::update(const ros::Time& time, const ros::Duration& period) {

    sensor_msgs::Imu imu_data = *imu_buffer_.readFromRT();
    tf::Quaternion q(
        imu_data.orientation.x, imu_data.orientation.y,
        imu_data.orientation.z, imu_data.orientation.w
    );
    tf::Matrix3x3 m(q);
    double cur_roll, cur_pitch, cur_yaw;
    m.getRPY(cur_roll, cur_pitch, cur_yaw);
    double pitch_rate = imu_data.angular_velocity.y;
   
    double vel_l = left_joint_.getVelocity();
    double vel_r = right_joint_.getVelocity();

    double current_linear_vel = (vel_l + vel_r) / 2.0 * wheel_radius_;
    double vel_error = cmd_linear_x_ - current_linear_vel;
    double target_pitch = velocity_pid_.computeCommand(vel_error, period); 

    double max_pitch = 0.25; 
    if (target_pitch > max_pitch) target_pitch = max_pitch;
    if (target_pitch < -max_pitch) target_pitch = -max_pitch;

    double pitch_error = cur_pitch - target_pitch;
    double base_effort = pid_controller_.computeCommand(pitch_error, pitch_rate, period);

    double current_yaw_rate = imu_data.angular_velocity.z;
    double yaw_error = cmd_angular_z_ - current_yaw_rate;
    double turn_effort = yaw_pid_.computeCommand(yaw_error, period);
    
    // 倒地后重置保护
    if (std::abs(cur_pitch) > 0.8) { 
        velocity_pid_.reset();
        pid_controller_.reset();
        yaw_pid_.reset();
        return;
    }

    double left_cmd = base_effort - turn_effort;
    double right_cmd = base_effort + turn_effort;

    double effort_limit = 5.0; 
    left_cmd = std::max(-effort_limit, std::min(effort_limit, left_cmd));
    right_cmd = std::max(-effort_limit, std::min(effort_limit, right_cmd));

    left_joint_.setCommand(left_cmd);
    right_joint_.setCommand(right_cmd);
}

void BalanceController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

    imu_buffer_.writeFromNonRT(*msg);
}

}

PLUGINLIB_EXPORT_CLASS(simble_box_bot_ns::BalanceController, controller_interface::ControllerBase)