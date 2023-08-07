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
    std::vector<std::string> robots;
    if (!n.getParam("/skills_executer/robots", robots))
    {
        ROS_ERRORE_RED_STREAM("No /skill_executer/robots param");
        exit(0);
    }

    std::vector<std::shared_ptr<skills_executer::SkillsExec>> skills_executers;
    for (const std::string robot: robots)
    {
        std::shared_ptr<skills_executer::SkillsExec> skills_exec = std::make_shared<skills_executer::SkillsExec> (n,robot);
        skills_executers.push_back(skills_exec);
    }

    skills_executer::BTToSkillsExecBridge bt_to_skills_exec_bridge(n,robots);

    while (ros::ok())
    {
        ros::Duration(1).sleep();
    }
}
