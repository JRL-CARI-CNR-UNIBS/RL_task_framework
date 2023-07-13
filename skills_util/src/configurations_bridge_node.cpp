/*****************************************************************************
** Includes
*****************************************************************************/

#include <skills_util/configurations_bridge.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc,argv,"configurations_bridge_node");
    if ( ! ros::master::check() )
    {
        printf("false");
        return false;
    }
    ros::start();
    ros::AsyncSpinner spinner(10);
    spinner.start();

    ros::NodeHandle n;

    skills_util::ConfigurationBridge config_bridge_node(n);

    while (ros::ok())
    {
        ros::Duration(1).sleep();
    }
}
