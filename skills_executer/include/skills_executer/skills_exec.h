#ifndef SKILLS_EXEC_H
#define SKILLS_EXEC_H

#include <ros/ros.h>
#include <skills_executer_msgs/SkillExecution.h>
#include <skills_arbitrator_msgs/SkillArbitration.h>
#include <skills_learning_msgs/SkillLearning.h>
#include <actionlib/client/simple_action_client.h>
#include <configuration_msgs/StartConfiguration.h>
#include <simple_touch_controller_msgs/SimpleTouchAction.h>
#include <simple_touch_controller_msgs/SimpleTouchActionGoal.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <manipulation_msgs/JobExecution.h>
#include <std_srvs/Trigger.h>
#include <relative_cartesian_controller_msgs/RelativeMoveAction.h>
#include <relative_cartesian_controller_msgs/RelativeMoveGoal.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ur_dashboard_msgs/Load.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <cmath>
#include <subscription_notifier/subscription_notifier.h>


namespace skills_executer
{

class SkillsExec
{
public:
    SkillsExec(const ros::NodeHandle & n);
    bool skillsExecution(skills_executer_msgs::SkillExecution::Request  &req,
                         skills_executer_msgs::SkillExecution::Response &res);

    bool changeConfig(std::string config_name);
    int urLoadProgram         (const std::string &action_name, const std::string &skill_name);
    int parallel2fGripperMove (const std::string &action_name, const std::string &skill_name);
    int robotiqGripperMove    (const std::string &action_name, const std::string &skill_name);
    int cartVel               (const std::string &action_name, const std::string &skill_name);
    int cartPos               (const std::string &action_name, const std::string &skill_name);
    int simpleTouch           (const std::string &action_name, const std::string &skill_name);
    double tf_distance (const std::string &reference_tf, const std::string &target_frame);
    void gripper_feedback     ();

    int reset_ur10e_ft_sensor ();
    double forceTopicCallback();
    double maxForce();
    void maxWrenchCalculation();
    template<typename T> void setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value);
    template<typename T> void setParam(const std::string &action_name, const std::string &param_name, const T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &param_name, T &param_value);

private:
    bool end_gripper_feedback_ = false;
    bool end_force_thread_ = false;
    double screw_accuracy_;
    double max_screw_accuracy_ = 10.0;
    double pi_ = 3.14159265358979323846;
    double max_force_ = 0.0;
    double gripper_tollerance_ = 0.01;
    std::string param_ns_ = "exec_params";
    std::string gripper_frame_ = "open_tip";
    ros::NodeHandle n_;
    ros::ServiceServer skill_exec_srv_;
    ros::ServiceClient start_config_clnt_;
    ros::ServiceClient skill_arbit_clnt_;
    ros::ServiceClient skill_explore_clnt_;
    std::shared_ptr<actionlib::SimpleActionClient<simple_touch_controller_msgs::SimpleTouchAction>>        touch_action_;
    std::shared_ptr<actionlib::SimpleActionClient<relative_cartesian_controller_msgs::RelativeMoveAction>> relative_move_action_;
    ros::Publisher twist_pub_;
    ros::Publisher gripper_move_pub_;

    std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>> wrench_sub_;
    std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> js_sub_;

    std::shared_ptr<std::thread> gripper_thread_;
    bool thread_esistence_ = false;

    std::string actual_action_name_, actual_skill_name_, last_pick_action_;

    std::string cart_vel_type_                 = "cartesian_velocity";
    std::string cart_pos_type_                 = "cartesian_position";
    std::string simple_touch_type_             = "simple_touch";
    std::string parallel_2f_gripper_move_type_ = "gripper_move";
//    std::string parallel_2f_gripper_move_type_ = "parallel_2f_gripper_move";
    std::string robotiq_gripper_move_type_     = "robotiq_gripper_move";
    std::string ur_load_program_               = "ur_load_program_";

    std::string watch_config_ = "watch";

    double desired_gripper_position_ = 0.79;
    tf::TransformListener tf_listener_;

};

template<typename T>
inline void SkillsExec::setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;

    ROS_INFO("Go to set param %s", param_str.c_str());
    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline void SkillsExec::setParam(const std::string &action_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+param_name;

    ROS_INFO("Go to set param %s", param_str.c_str());
    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline bool SkillsExec::getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        ROS_WARN("%s not exist", param_str.c_str());
        return false;
    }
    return true;
}

template<typename T>
inline bool SkillsExec::getParam(const std::string &action_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        ROS_WARN("%s not exist", param_str.c_str());
        return false;
    }
    return true;
}
} // end namespace skills_executer

#endif // SKILLS_EXEC_H
