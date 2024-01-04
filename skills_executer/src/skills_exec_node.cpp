/*****************************************************************************
** Includes
*****************************************************************************/

#include <skills_executer/skills_exec.h>
#include <skills_executer/bt_to_skills_exec_bridge.h>

/*****************************************************************************
** Main
*****************************************************************************/


int main(int argc, char **argv)
{  
    ros::init(argc,argv,"skills_exec_node");
    if ( ! ros::master::check() )
    {
        printf("false");
        return false;
    }
    ros::start();
    ros::AsyncSpinner spinner(10);
    spinner.start();

    ros::NodeHandle n;
    XmlRpc::XmlRpcValue robots_param;
    if (!n.getParam("/skills_executer/robots", robots_param))
    {
        ROS_ERROR_RED_STREAM("No /skills_executer/robots param");
        exit(0);
    }

    if (robots_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        ROS_ERROR("/skills_executer/robots is not a struct");
        exit(0);
    }

    std::vector<std::string> robots_names = skills_util::getMemberByXml(robots_param);

    std::vector<std::shared_ptr<skills_executer::SkillsExec>> skills_executers;
    for (const std::string robot_name: robots_names)
    {
        std::shared_ptr<skills_executer::SkillsExec> skills_exec = std::make_shared<skills_executer::SkillsExec> (n,robot_name);
        skills_executers.push_back(skills_exec);
    }

    skills_executer::BTToSkillsExecBridge bt_to_skills_exec_bridge(n,robots_names);

    while (ros::ok())
    {
        ros::Duration(0.1).sleep();
    }
}
