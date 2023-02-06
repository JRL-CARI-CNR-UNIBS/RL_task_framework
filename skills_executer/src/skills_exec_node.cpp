/*****************************************************************************
** Includes
*****************************************************************************/

#include <skills_executer/skills_exec.h>

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
    skills_executer::SkillsExec skills_exec_(n);

    while (ros::ok())
    {
        ros::Duration(1).sleep();
    }
}
