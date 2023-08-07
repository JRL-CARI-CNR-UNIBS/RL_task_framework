#ifndef BTTOSKILLSEXECBRIDGE_H
#define BTTOSKILLSEXECBRIDGE_H

#include <ros/ros.h>
#include <skills_executer_msgs/SkillExecution.h>
#include <skills_executer_msgs/RobotSkillExecution.h>

namespace skills_executer
{

class BTToSkillsExecBridge
{
public:
    BTToSkillsExecBridge(const ros::NodeHandle &n, const std::vector<std::string> &robots);
    bool skillsExecution(skills_executer_msgs::RobotSkillExecution::Request  &req,
                         skills_executer_msgs::RobotSkillExecution::Response &res);

private:
    ros::NodeHandle n_;
    ros::ServiceServer skill_exec_srv_;
    std::map<std::string,ros::ServiceClient> skill_exec_clnts_;
};

} // end namespace skills_executer

#endif // BTTOSKILLSEXECBRIDGE_H
