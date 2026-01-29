#include "simble_box_bot/lqr_controller.h"
#include <tf/tf.h>
#include <cmath>

namespace simble_box_bot_ns {

bool LQRBalanceController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
   
    std::string left_joint_name, right_joint_name;
    if (!nh.getParam("left_wheel_name", left_joint_name) || !nh.getParam("right_wheel_name", right_joint_name)) {
        ROS_ERROR("No Joint Names found in param server");
        return false;
    }
    left_joint_ = hw->getHandle(left_joint_name);
    right_joint_ = hw->getHandle(right_joint_name);

    double y_p, y_i, y_d;
    nh.param("yaw_kp", y_p, 2.0); 
    nh.param("yaw_ki", y_i, 0.0);
    nh.param("yaw_kd", y_d, 0.1);
    yaw_pid_.initPid(y_p, y_i, y_d, 3.0, -3.0);

    nh.param("k1", k1, 0.0);
    nh.param("k2", k2, 0.0);
    nh.param("k3", k3, 0.0);
    nh.param("k4", k4, 0.0);

    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu_data", 1, &LQRBalanceController::imuCallback, this);
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &LQRBalanceController::cmdVelCallback, this);

    return true;
}


void LQRBalanceController::starting(const ros::Time& time) {
    cmd_linear_x_ = 0.0;
    cmd_yaw_rate_ = 0.0;
    target_pitch_ = 0.0;
    left_joint_.setCommand(0.0);
    right_joint_.setCommand(0.0);
    yaw_pid_.reset();
}

void LQRBalanceController::update(const ros::Time& time, const ros::Duration& period) {

    sensor_msgs::Imu imu_data = *imu_buffer_.readFromRT();
    tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double dt = period.toSec();
    current_pitch_ = pitch;             
    current_pitch_dot_ = imu_data.angular_velocity.y; 
    // 获取电机当前的累积旋转角度 (rad)
    double pl_rad = left_joint_.getPosition();
    double pr_rad = right_joint_.getPosition();
    // 获取电机当前的旋转角速度 (rad/s)
    double vl_rad = left_joint_.getVelocity();
    double vr_rad = right_joint_.getVelocity();
    // 计算中心点的位移和速度
    current_pos_ = (pl_rad + pr_rad) * wheel_radius_ / 2.0; 
    target_pos_ += cmd_linear_x_ * dt;
    double pos_error = target_pos_ - current_pos_;

    current_vel_ = (vl_rad + vr_rad) * wheel_radius_ / 2.0; 
    target_linear_vel_ = cmd_linear_x_;
    double vel_error = target_linear_vel_ - current_vel_;

    double pitch_error = target_pitch_ - current_pitch_;
    double pitch_dot_error = target_pitch_dot_ - current_pitch_dot_;

    double base_effort = (k1 * pos_error + k2 * vel_error + k3 * pitch_error + k4 * pitch_dot_error);
    double turn_effort = yaw_pid_.computeCommand(cmd_yaw_rate_ - imu_data.angular_velocity.z, period);
    

    double left_effort = base_effort / 2.0 - turn_effort;
    double right_effort = base_effort / 2.0 + turn_effort;

    // 倒地保护
    // if (std::abs(current_pitch_) > 0.8) { 
    //     left_effort = 0.0; 
    //     right_effort = 0.0; 
    //     target_pos_ = 0.0;
    //     yaw_pid_.reset();
    // }

    left_joint_.setCommand(left_effort);
    right_joint_.setCommand(right_effort);
    ROS_INFO_THROTTLE(0.5, "Angle: %.2f, Computed Torque u: %.2f", current_pitch_, base_effort);
}

void LQRBalanceController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_buffer_.writeFromNonRT(*msg);
}

void LQRBalanceController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_linear_x_ = msg->linear.x;
    cmd_yaw_rate_ = msg->angular.z;
}

// void LQRBalanceController::computeDynamics() {
//     double M = m_c_;                // 车体质量
//     double m = 2.0 * m_w_;          // 双轮质量
//     double J_c = I_c_;              // 车体俯仰惯量 
//     double J_w = 2.0 * I_w_;        // 双轮转动惯量
//     double R = R_;                  // 轮半径
//     double H_com = L_;              // 重心高度
//     double dynamics_l = H_com - R;
//     double g = 9.81;

//     // 2. 计算中间变量,拉格朗日方程的线性化推导

//     double mass_factor = M + m + J_w / (R * R); 
//     double denominator = J_c * mass_factor + M * dynamics_l * dynamics_l * (m + J_w / (R * R)); 

//     double det = (M*dynamics_l)*(M*dynamics_l) - (J_c + M*dynamics_l*dynamics_l)*(M + m + J_w/(R*R)); 
    
//     // 重算常用系数
//     double I_total = J_c + M*dynamics_l*dynamics_l;   // 车体绕轴的惯量
//     double M_total = M + m + J_w/(R*R); // 等效平移质量
//     double denom = I_total * M_total - (M*dynamics_l)*(M*dynamics_l); 

//     A_.setZero();
//     B_.setZero();

//     // Row 0: Theta_dot (d/dt theta = theta_dot)
//     A_(0, 1) = 1.0;
//     // Row 1: Theta_ddot
//     A_(1, 0) = (M_total * M * g * dynamics_l) / denom;
//     A_(1, 3) = 0.0; 
//     // Row 2: X_dot
//     A_(2, 3) = 1.0;
//     // Row 3: X_ddot
//     A_(3, 0) = (M * dynamics_l * M * g * dynamics_l) / denom; 
//     A_(3, 0) = (M * M * g * dynamics_l  * dynamics_l ) / denom;

//     // B 矩阵
//     B_(1) = -(M_total) / denom;     
//     B_(3) = (M * dynamics_l + I_total / R) / denom; 
// }


// void LQRBalanceController::computeLQR() {
//     ROS_INFO("Computing LQR gain...");
    
//     if (!A_.allFinite() || !B_.allFinite()) {
//         ROS_ERROR_STREAM("Dynamics matrix A or B contains NaN/Inf!\nA:\n" << A_ << "\nB:\n" << B_);
//         return;
//     }
//     Eigen::Matrix4d P = Q_;
    
//     double dt = 1e-6;     
//     int max_iter = 100000; 
//     double inv_R = 1.0 / R_cost_;
//     double diff = 0.0;

//     for(int i=0; i<max_iter; ++i) {
//         Eigen::Matrix4d P_dot = A_.transpose() * P + P * A_ - P * B_ * inv_R * B_.transpose() * P + Q_;
//         P += P_dot * dt; 
//         diff = P_dot.norm();
//         if (diff < 1e-4) {
//             ROS_INFO("LQR Solver converged at iteration %d, error: %f", i, diff);
//             break;
//         }
//     }
    
//     if (diff > 1.0) {
//         ROS_ERROR("LQR Solver DIVERGED! Parameters are too aggressive for this simple solver.");
//         ROS_ERROR("Try increasing lqr_r or reducing lqr_q_*.");
//     }

//     K_ = inv_R * B_.transpose() * P;
//     if (!K_.allFinite()) {
//         ROS_ERROR("Computed LQR Gain contains NaN!");
//         K_ << 0, 0, 0, 0; 
//     } else {
//         ROS_INFO_STREAM("LQR Gain K: " << K_);
//     }
// }

} // namespace

PLUGINLIB_EXPORT_CLASS(simble_box_bot_ns::LQRBalanceController, controller_interface::ControllerBase)