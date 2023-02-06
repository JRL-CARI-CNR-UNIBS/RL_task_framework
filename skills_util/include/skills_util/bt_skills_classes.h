#ifndef BT_SKILLS_CLASSES_H
#define BT_SKILLS_CLASSES_H

#include <ros/ros.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <skills_executer_msgs/SkillExecution.h>
#include <skills_learning_msgs/SkillLearning.h>
#include <skills_learning_msgs/SkillExplore.h>
#include <skills_arbitrator_msgs/SkillArbitration.h>

class SkillActionNode : public BT::SyncActionNode
{
public:
    SkillActionNode(const std::string& name);

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_exec_clnt_;
};

class SkillExecutionNode : public BT::SyncActionNode
{
public:
    SkillExecutionNode(const std::string& name);

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_exec_clnt_;
};

class ActionLearningNode : public BT::SyncActionNode
{
public:
    ActionLearningNode(const std::string& name);

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_learn_clnt_;
};

class ActionExploretionNode : public BT::SyncActionNode
{
public:
    ActionExploretionNode(const std::string& name);

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_explore_clnt_;
};

class ActionArbitrationNode : public BT::SyncActionNode
{
public:
    ActionArbitrationNode(const std::string& name);

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_arbit_clnt_;
};

namespace bt_skills_classes {
std::vector<std::string> skillNames(const std::string& action_skill_name);
}

#endif // SKILLS_CLASSES_H
