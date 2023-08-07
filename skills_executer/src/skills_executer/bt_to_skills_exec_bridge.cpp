#include <skills_executer/bt_to_skills_exec_bridge.h>

namespace skills_executer
{

BTToSkillsExecBridge::BTToSkillsExecBridge(const ros::NodeHandle &n, const std::vector<std::string> &robots) : n_(n)
{
    for (std::string robot: robots)
    {
        ros::ServiceClient skill_exec_clnt = n_.serviceClient<skills_executer_msgs::SkillExecution>("/"+robot+"/skills_exec/execute_skill");
        ROS_WARN_STREAM("Waiting for "<<skill_exec_clnt.getService());
        skill_exec_clnt.waitForExistence();
        ROS_WARN_STREAM("Connection ok");
        skill_exec_clnts_.insert(std::make_pair(robot,skill_exec_clnt));
    }

    skill_exec_srv_ = n_.advertiseService("/skills_exec/execute_skill", &BTToSkillsExecBridge::skillsExecution, this);
}

bool BTToSkillsExecBridge::skillsExecution(skills_executer_msgs::RobotSkillExecution::Request  &req,
                                           skills_executer_msgs::RobotSkillExecution::Response &res)
{
    if (req.robot_name.empty())
    {
        ROS_ERROR_STREAM("Robot param empty");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    skills_executer_msgs::SkillExecution skill_exec_srv;
    skill_exec_srv.request.action_name = req.action_name;
    skill_exec_srv.request.skill_name  = req.skill_name;

    if (!skill_exec_clnts_.at(req.robot_name).call(skill_exec_srv))
    {
        ROS_ERROR_STREAM("Unable to contact "<<skill_exec_clnts_.at(req.robot_name).getService()<<" server");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    return skill_exec_srv.response.result;
}
} // end namespace skills_executer
