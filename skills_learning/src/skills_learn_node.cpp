/*****************************************************************************
** Includes
*****************************************************************************/

#include <skills_learning/skills_learn.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc,argv,"skills_learn_node");
    if ( ! ros::master::check() )
    {
        printf("false");
        return false;
    }
    ros::start();
    ros::NodeHandle n;
    skills_learning::SkillsLearn skills_learn_(n);

    ros::spin();
}
