#ifndef SKILLS_ARBIT_H
#define SKILLS_ARBIT_H

#include <ros/ros.h>
#include <skills_arbitrator_msgs/SkillArbitration.h>
#include <skills_util/log.h>
#include <skills_util/util_functions.h>

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
    std::string exec_param_ns_, arbit_param_ns_;

    std::map<std::string,std::map<std::string,std::map<std::string,double>>> skill_evaluation_parameters_;
    std::map<std::string,std::map<std::string,double>> action_evaluation_parameters_;
};

template<typename T>
inline bool SkillsArbit::getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+exec_param_ns_+"/actions/"+action_name+"/skills/"+skill_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        return false;
    }
    return true;
}

template<typename T>
inline bool SkillsArbit::getParam(const std::string &action_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+exec_param_ns_+"/actions/"+action_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        return false;
    }
    return true;
}

template<typename T>
inline void SkillsArbit::setParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+exec_param_ns_+"/actions/"+action_name+"/skills/"+skill_name+"/"+param_name;

    n_.setParam(param_str, param_value);
    return;
}

template<typename T>
inline void SkillsArbit::setParam(const std::string &action_name, const std::string &param_name, const T &param_value)
{
    std::string param_str = "/"+exec_param_ns_+"/actions/"+action_name+"/"+param_name;

    n_.setParam(param_str, param_value);
    return;
}

} // end namespace skills_arbitrator

#endif // SKILLS_ARBIT_H
