#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>
#include <math.h>

namespace hero_controllers {

    HeroChassisController::HeroChassisController()
            : linear_x_(0.0), linear_y_(0.0), angular_z_(0.0), x_pos_(0.0), y_pos_(0.0), theta_(0.0) {}

    HeroChassisController::~HeroChassisController()
    {
        sub_command_.shutdown();
    }

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &n)
    {
        // Load parameters
        if (!n.getParam("front_left_wheel", front_left_wheel_) ||
            !n.getParam("front_right_wheel", front_right_wheel_) ||
            !n.getParam("rear_left_wheel", rear_left_wheel_) ||
            !n.getParam("rear_right_wheel", rear_right_wheel_)) {
            ROS_ERROR("Wheel names not specified");
            return false;
        }

        if (!n.getParam("wheel_base", wheel_base_) ||
            !n.getParam("wheel_track", wheel_track_) ||
            !n.getParam("wheel_radius", wheel_radius_)) {
            ROS_ERROR("Chassis parameters not specified");
            return false;
        }

        // Get joint handles
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        rear_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        rear_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        // Initialize PID controllers
        if (!pid_front_left_.init(ros::NodeHandle(n, "pid/front_left")) ||
            !pid_front_right_.init(ros::NodeHandle(n, "pid/front_right")) ||
            !pid_rear_left_.init(ros::NodeHandle(n, "pid/rear_left")) ||
            !pid_rear_right_.init(ros::NodeHandle(n, "pid/rear_right"))) {
            return false;
        }

        // Subscribe to velocity commands
        sub_command_ = n.subscribe("/cmd_vel", 1, &HeroChassisController::setCommandCB, this);

        // Initialize odometry publisher
        odom_pub_ = n.advertise<nav_msgs::Odometry>("/odom", 50);
        last_time_ = ros::Time::now();

        return true;
    }

    void HeroChassisController::starting(const ros::Time &time)
    {
        linear_x_ = 0.0;
        linear_y_ = 0.0;
        angular_z_ = 0.0;
        x_pos_ = 0.0;
        y_pos_ = 0.0;
        theta_ = 0.0;

        pid_front_left_.reset();
        pid_front_right_.reset();
        pid_rear_left_.reset();
        pid_rear_right_.reset();
    }

    void HeroChassisController::setCommandCB(const geometry_msgs::TwistConstPtr &msg)
    {
        linear_x_ = msg->linear.x;
        linear_y_ = msg->linear.y;
        angular_z_ = msg->angular.z;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period)
    {
        // Compute desired wheel velocities
        double vx = linear_x_;
        double vy = linear_y_;
        double wz = angular_z_;

        double l = wheel_base_;
        double w = wheel_track_;
        double r = wheel_radius_;

        double front_left_velocity = (vx - vy - (l + w) * wz) / r;
        double front_right_velocity = (vx + vy + (l + w) * wz) / r;
        double rear_left_velocity = (vx + vy - (l + w) * wz) / r;
        double rear_right_velocity = (vx - vy + (l + w) * wz) / r;

        // Compute errors and efforts
        double error_fl = front_left_velocity - front_left_joint_.getVelocity();
        double error_fr = front_right_velocity - front_right_joint_.getVelocity();
        double error_rl = rear_left_velocity - rear_left_joint_.getVelocity();
        double error_rr = rear_right_velocity - rear_right_joint_.getVelocity();

        double effort_fl = pid_front_left_.computeCommand(error_fl, period);
        double effort_fr = pid_front_right_.computeCommand(error_fr, period);
        double effort_rl = pid_rear_left_.computeCommand(error_rl, period);
        double effort_rr = pid_rear_right_.computeCommand(error_rr, period);

        // Set wheel efforts
        front_left_joint_.setCommand(effort_fl);
        front_right_joint_.setCommand(effort_fr);
        rear_left_joint_.setCommand(effort_rl);
        rear_right_joint_.setCommand(effort_rr);

        // Compute odometry
        computeOdometry(period);
    }

    void HeroChassisController::computeOdometry(const ros::Duration &period)
    {
        double r = wheel_radius_;
        double l = wheel_base_;
        double w = wheel_track_;

        // Get wheel velocities
        double v_fl = front_left_joint_.getVelocity() * r;
        double v_fr = front_right_joint_.getVelocity() * r;
        double v_rl = rear_left_joint_.getVelocity() * r;
        double v_rr = rear_right_joint_.getVelocity() * r;

        // Compute chassis velocity
        double vx = (v_fl + v_fr + v_rl + v_rr) / 4.0;
        double vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
        double wz = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (l + w));

        // Update position
        double dt = period.toSec();
        x_pos_ += (vx * cos(theta_) - vy * sin(theta_)) * dt;
        y_pos_ += (vx * sin(theta_) + vy * cos(theta_)) * dt;
        theta_ += wz * dt;

        // Publish odometry
        ros::Time current_time = ros::Time::now();
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x_pos_;
        odom.pose.pose.position.y = y_pos_;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        odom_pub_.publish(odom);

        // Publish TF
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_pos_;
        odom_trans.transform.translation.y = y_pos_;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta_);

        odom_broadcaster_.sendTransform(odom_trans);
    }

} // namespace hero_controllers

PLUGINLIB_EXPORT_CLASS(hero_controllers::HeroChassisController, controller_interface::ControllerBase)
