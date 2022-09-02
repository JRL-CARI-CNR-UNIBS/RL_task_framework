#ifndef SKILLS_ARBIT_H
#define SKILLS_ARBIT_H

#include <ros/ros.h>
#include <skills_arbitrator_msgs/SkillArbitration.h>

namespace skills_arbitrator
{

class SkillsArbit
{
public:
    SkillsArbit(const ros::NodeHandle & n);
    bool skillsArbitration(skills_arbitrator_msgs::SkillArbitration::Request  &req,
                           skills_arbitrator_msgs::SkillArbitration::Response &res);

    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &param_name, T &param_value);
    template<typename T> void setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value);
    template<typename T> void setParam(const std::string &action_name, const std::string &param_name, const T &param_value);

private:
    ros::NodeHandle n_;
    ros::ServiceServer skill_arbit_srv_;
    std::string param_ns_ = "exec_param";

    double fail_reward_ = 10000000.0;

    std::string cart_vel_type_                 = "cartesian_velocity";
    std::string cart_pos_type_                 = "cartesian_position";
    std::string simple_touch_type_             = "simple_touch";
    std::string parallel_2f_gripper_move_type_ = "gripper_move";
//    std::string parallel_2f_gripper_move_type_ = "parallel_2f_gripper_move";
    std::string robotiq_gripper_move_type_     = "robotiq_gripper_move";
    std::string ur_load_program_               = "ur_load_program_";

    std::vector<std::string> skill_types = {"cartesian_velocity",
                                            "cartesian_position",
//                                            "simple_touch",
                                            "gripper_move",
                                            "robotiq_gripper_move"
                                           };
    std::map<std::string,std::vector<std::string>> action_evaluation_parameters_ = {
        {"pick",  {"traveled_distance", "fail"} },
        {"place", {"traveled_distance", "fail"} },
    };

    std::map<std::string,std::vector<double>> action_evaluation_weight_ = {
        {"pick",  {1.0, 100000} },
        {"place", {1.0, 100000} },
    };

    std::map<std::string,std::vector<std::string>> skill_evaluation_parameters_ = {
        {"cartesian_velocity",   {"duration", "max_force"} },
        {"cartesian_position",   {"duration", "max_force"} },
        {"simple_touch",         {"duration", "max_force"} },
        {"gripper_move",         {"torque", "fail"}        },
        {"robotiq_gripper_move", {"torque", "fail"}        },
    };

    std::map<std::string,std::vector<double>> skill_evaluation_weight_ = {
        {"cartesian_velocity",   {1.0, 0.5}    },
        {"cartesian_position",   {1.0, 0.5}    },
        {"simple_touch",         {0.5, 100000} },
        {"gripper_move",         {1.0, fail_reward_}    },
        {"robotiq_gripper_move", {0.5, fail_reward_} }
    };

};

template<typename T>
inline bool SkillsArbit::getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        ROS_WARN("%s not set", param_str.c_str());
        return false;
    }
    return true;
}

template<typename T>
inline bool SkillsArbit::getParam(const std::string &action_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        ROS_WARN("%s not set", param_str.c_str());
        return false;
    }
    return true;
}

template<typename T>
inline void SkillsArbit::setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;

    ROS_INFO("Go to set param %s", param_str.c_str());
    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline void SkillsArbit::setParam(const std::string &action_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+param_name;

    ROS_INFO("Go to set param %s", param_str.c_str());
    n_.setParam(param_str, param_value);
    return;
}

} // end namespace skills_arbitrator

#endif // SKILLS_ARBIT_H
