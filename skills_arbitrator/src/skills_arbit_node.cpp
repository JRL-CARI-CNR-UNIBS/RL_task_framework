/*****************************************************************************
** Includes
*****************************************************************************/

#include <skills_arbitrator/skills_arbit.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc,argv,"skills_arbit_node");
    if ( ! ros::master::check() )
    {
        printf("false");
        return false;
    }
    ros::start();
    ros::NodeHandle n;
    skills_arbitrator::SkillsArbit skills_arbit_(n);

    ros::spin();
}
