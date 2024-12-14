#ifndef HERO_CONTROLLERS_HERO_CHASSIS_CONTROLLER_H
#define HERO_CONTROLLERS_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

namespace hero_controllers {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        HeroChassisController();
        ~HeroChassisController();

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &n) override;
        void starting(const ros::Time &time) override;
        void update(const ros::Time &time, const ros::Duration &period) override;

    private:
        void setCommandCB(const geometry_msgs::TwistConstPtr &msg);
        void computeOdometry(const ros::Duration &period);

        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle rear_left_joint_;
        hardware_interface::JointHandle rear_right_joint_;

        std::string front_left_wheel_;
        std::string front_right_wheel_;
        std::string rear_left_wheel_;
        std::string rear_right_wheel_;

        control_toolbox::Pid pid_front_left_;
        control_toolbox::Pid pid_front_right_;
        control_toolbox::Pid pid_rear_left_;
        control_toolbox::Pid pid_rear_right_;

        ros::Subscriber sub_command_;
        ros::Publisher odom_pub_;
        tf::TransformBroadcaster odom_broadcaster_;

        double linear_x_;
        double linear_y_;
        double angular_z_;

        double x_pos_, y_pos_, theta_;
        ros::Time last_time_;

        double wheel_base_;
        double wheel_track_;
        double wheel_radius_;
    };

} // namespace hero_controllers

#endif // HERO_CONTROLLERS_HERO_CHASSIS_CONTROLLER_H
