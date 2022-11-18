#ifndef SKILLS_LEARN_H
#define SKILLS_LEARN_H

#include <ros/ros.h>
#include <ros/console.h>
#include <skills_learning_msgs/SkillLearning.h>
#include <skills_learning_msgs/SkillExplore.h>
#include <skills_util/log.h>

namespace skills_learning
{

class SkillsLearn
{
public:
    SkillsLearn(const ros::NodeHandle & n);
    bool skillsExplore (skills_learning_msgs::SkillExplore::Request  &req,
                        skills_learning_msgs::SkillExplore::Response &res);
    bool skillsLearning(skills_learning_msgs::SkillLearning::Request  &req,
                        skills_learning_msgs::SkillLearning::Response &res);

    int explore (const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name);
    int learning(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name);

    void printNewOldParam (std::string name, std::vector<double> param, std::vector<double> param_old);
    void printNewOldParam (std::string name, std::vector<int> param, std::vector<int> param_old);
    void printArrayParam (std::string name, std::vector<int> param);
    void printArrayParam (std::string name, std::vector<double> param);

    template<typename T> void setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value);
    template<typename T> void setParam(const std::string &action_name, const std::string &param_name, const T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &param_name, T &param_value);

private:
    std::string param_ns_ = "RL_params";
    ros::NodeHandle n_;

    double total_reward_, total_reward_old_;

    std::map<std::string,std::vector<std::string>> skill_execution_parameters_ = {
        {"cartesian_velocity",   { "position", "quaternion", "linear_velocity", "angular_velocity"} },
        {"cartesian_position",   { "position", "quaternion", "linear_velocity", "angular_velocity"} },
        {"simple_touch",         { "position", "quaternion", "linear_velocity", "angular_velocity"} },
        {"gripper_move",         { "torque"} },
        {"robotiq_gripper_move", { "torque"} },
        {"go_to",                { } }
    };

//    std::vector<std::string> cart_pos_params_                 = { "position", "quaternion", "linear_velocity", "angular_velocity"};
//    std::vector<std::string> cart_vel_params_                 = { "position", "quaternion", "linear_velocity", "angular_velocity"};
//    std::vector<std::string> simple_touch_params_             = { "position", "quaternion", "linear_velocity", "angular_velocity"};
//    std::vector<std::string> parallel_2f_gripper_move_params_ = { "position", "quaternion", "linear_velocity", "angular_velocity", "fail"};
//    std::vector<std::string> robotiq_gripper_move_params_     = { "position", "quaternion", "linear_velocity", "angular_velocity", "fail"};
//    std::vector<std::string> ur_load_program_params_          = { "position", "quaternion", "linear_velocity", "angular_velocity"};
    std::string cart_vel_type_                 = "cartesian_velocity";
    std::string cart_pos_type_                 = "cartesian_position";
    std::string simple_touch_type_             = "simple_touch";
    std::string parallel_2f_gripper_move_type_ = "gripper_move";
//    std::string parallel_2f_gripper_move_type_ = "parallel_2f_gripper_move";
    std::string robotiq_gripper_move_type_     = "robotiq_gripper_move";
    std::string ur_load_program_               = "ur_load_program_";

    ros::ServiceServer skill_learn_srv_;
    ros::ServiceServer skill_explore_srv_;
};

template<typename T>
inline void SkillsLearn::setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;
    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline void SkillsLearn::setParam(const std::string &action_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+param_name;
    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline bool SkillsLearn::getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        return false;
    }
    return true;
}

template<typename T>
inline bool SkillsLearn::getParam(const std::string &action_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        return false;
    }
    return true;
}

} // end namespace skills_learning

#endif // SKILLS_LEARN_H
