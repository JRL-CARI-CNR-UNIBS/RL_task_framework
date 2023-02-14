/*****************************************************************************
** Includes
*****************************************************************************/

#include <skills_util/bt_exec.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc,argv,"bt_exec_node");
    if ( ! ros::master::check() )
    {
        printf("false");
        return false;
    }
    ros::start();
    ros::AsyncSpinner spinner(10);
    spinner.start();

    ros::NodeHandle n;
    skills_util::BTExec bt_exec_(n);

    while (ros::ok())
    {
        ros::Duration(1).sleep();
    }
}
