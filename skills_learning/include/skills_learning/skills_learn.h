#ifndef SKILLS_LEARN_H
#define SKILLS_LEARN_H

#include <ros/ros.h>
#include <ros/console.h>
#include <skills_learning_msgs/SkillLearning.h>
#include <skills_learning_msgs/SkillExplore.h>
#include <skills_util/log.h>
#include <skills_util/util_functions.h>

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

    int explore_variable_range (const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name);
    int learning_latest_more_weight(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name);


    void printNewOldParam (std::string name, std::vector<double> param, std::vector<double> param_old);
    void printNewOldParam (std::string name, std::vector<int> param, std::vector<int> param_old);
    void printArrayParam (std::string name, std::vector<int> param);
    void printArrayParam (std::string name, std::vector<double> param);

    template<typename T> void setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value);
    template<typename T> void setParam(const std::string &action_name, const std::string &param_name, const T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);
    template<typename T> bool getParam(const std::string &action_name, const std::string &param_name, T &param_value);

private:
    std::string param_ns_;
    ros::NodeHandle n_;

    double total_reward_, total_reward_old_;

    std::map<std::string,std::vector<std::string>> skill_execution_parameters_;

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
