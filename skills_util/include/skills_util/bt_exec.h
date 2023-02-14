#ifndef BT_EXEC_H
#define BT_EXEC_H

#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <skills_util/bt_skills_classes.h>
#include <skills_util_msgs/RunTree.h>
#include <filesystem>

namespace skills_util
{

class BTExec
{
public:
    BTExec(const ros::NodeHandle & n);
    bool runTree(skills_util_msgs::RunTree::Request  &req,
                 skills_util_msgs::RunTree::Response &res);

private:
    BT::BehaviorTreeFactory factory_;
    ros::ServiceServer bt_exec_srv_;
    ros::NodeHandle n_;
};

} // end namespace skills_util

#endif // BT_EXEC_H
