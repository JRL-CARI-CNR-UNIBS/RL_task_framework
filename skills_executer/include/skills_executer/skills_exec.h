#ifndef SKILLS_EXEC_H
#define SKILLS_EXEC_H

#include <ros/ros.h>
#include <skills_executer_msgs/SkillExecution.h>
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
#include <tf_conversions/tf_eigen.h>
#include <ur_dashboard_msgs/Load.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <cmath>
#include <subscription_notifier/subscription_notifier.h>
#include <skills_util/log.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>
#include <parallel_2f_gripper/MoveGripper.h>
#include <pybullet_utils/SensorReset.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <skills_util_msgs/ChangeConfig.h>

//fjt part
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
//end

#include <ik_solver_msgs/GetIk.h>
#include <ik_solver_msgs/Configuration.h>
#include <ik_solver_msgs/IkSolution.h>

namespace skills_executer
{

class SkillsExec
{
public:
    SkillsExec(const ros::NodeHandle & n, const std::string & name);
    bool skillsExecution(skills_executer_msgs::SkillExecution::Request  &req,
                         skills_executer_msgs::SkillExecution::Response &res);

    bool changeConfig         (const std::string config_name);
    int urLoadProgram         (const std::string &action_name, const std::string &skill_name);
    int robotiqGripperMove    (const std::string &action_name, const std::string &skill_name);
    int cartVel               (const std::string &action_name, const std::string &skill_name);
    int simpleTouch           (const std::string &action_name, const std::string &skill_name);
    int cartPos               (const std::string &action_name, const std::string &skill_name, const int &move_type);
    int move_to               (const std::string &action_name, const std::string &skill_name, const int &move_type);
    int parallel2fGripperMove (const std::string &action_name, const std::string &skill_name);
    int releaseEndEffector    (const std::string &action_name, const std::string &skill_name);
    int attachEndEffector     (const std::string &action_name, const std::string &skill_name);
    int joint_move_to         (const std::string &action_name, const std::string &skill_name);
    double tf_distance        (const std::string &reference_tf, const std::string &target_frame);

    void gripper_feedback     ();

    int reset_ft_sensor();
    int reset_pybullet_ft_sensor();
    int reset_ur10e_ft_sensor ();
    double forceTopicCallback();
    double maxForce();
    void maxWrenchCalculation();

    bool setReferenceEndEffectorFrame();
    bool setSensoredJoint();
    bool setAttachedLinkName();
    bool setEndEffectorTouchLinks();


    template<typename T> void setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value);
    template<typename T> void setParam(const std::string &action_name, const std::string &param_name, const T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &param_name, T &param_value);


private:
    bool end_gripper_feedback_ = false;
    bool end_force_thread_ = false;
    bool contact_ = false;
    bool use_pybullet_ = false;
    bool use_ur_ = false;
    bool use_change_config_bridge_ = false;
    double max_force_ = 0.0;
    double gripper_tollerance_ = 0.01;
    double max_force_variation_ = 500;
    double minimum_gripper_force_ = 5;
    double parallel_2f_gripper_closed_position_ = -0.79;

    tf::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster tf_br_;
    std::string robot_name_;
    std::string param_ns_ = "RL_params";
    std::string end_link_frame_;
    std::string reference_end_effector_frame_;
    std::string sensored_joint_;
    std::string attached_link_name_;
    std::vector<std::string> end_effector_touch_links_;
    ros::NodeHandle n_;
    ros::ServiceServer skill_exec_srv_;
    ros::ServiceClient parallel_gripper_move_clnt_;
    ros::ServiceClient sensor_reset_clnt_;
    ros::ServiceClient change_config_clnt_;
    ros::ServiceClient get_ik_clnt_;

    std::shared_ptr<actionlib::SimpleActionClient<simple_touch_controller_msgs::SimpleTouchAction>>        touch_action_;
    std::shared_ptr<actionlib::SimpleActionClient<relative_cartesian_controller_msgs::RelativeMoveAction>> relative_move_action_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    ros::Publisher twist_pub_;
    ros::Publisher gripper_move_pub_;

    std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>> wrench_sub_;
    std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> js_sub_;

    std::shared_ptr<std::thread> gripper_thread_;
    bool thread_esistence_ = false;

    std::string current_action_name_, current_skill_name_, last_pick_action_, current_grasped_object_;

    std::string cart_vel_type_                 = "cartesian_velocity";
    std::string cart_pos_type_                 = "cartesian_position";
    std::string cart_pos_to_type_              = "cart_pos_to";
    std::string simple_touch_type_             = "simple_touch";
    std::string parallel_2f_gripper_move_type_ = "parallel_gripper_move";
//    std::string parallel_2f_gripper_move_type_ = "parallel_2f_gripper_move";
    std::string robotiq_gripper_move_type_     = "robotiq_gripper_move";
    std::string ur_load_program_               = "ur_load_program_";
    std::string move_to_type_                  = "move_to";
    std::string linear_move_type_              = "linear_move";
    std::string linear_move_to_type_           = "linear_move_to";
    std::string joint_move_to_type_            = "joint_move_to";
    std::string release_end_effector_type_      = "release_end_effector";
    std::string attach_end_effector_type_      = "attach_end_effector";

    std::string watch_config_ = "watch";

    Eigen::Affine3d T_gripper_to_end_link_;
    moveit::planning_interface::MoveGroupInterface::Plan moveit_plan_;


//    fjt part
    double trajectory_time_tollerance_ = 2;
    std::vector<double> trajectory_joint_tollerance_;
    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_ac_;
//    end
};

template<typename T>
inline void SkillsExec::setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;

    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline void SkillsExec::setParam(const std::string &action_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+param_name;

    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline bool SkillsExec::getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
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
        return false;
    }
    return true;
}
} // end namespace skills_executer

#endif // SKILLS_EXEC_H
