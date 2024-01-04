#ifndef BT_SKILLS_CLASSES_H
#define BT_SKILLS_CLASSES_H

#include <ros/ros.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <skills_executer_msgs/RobotSkillExecution.h>
#include <skills_learning_msgs/SkillLearning.h>
#include <skills_learning_msgs/SkillExplore.h>
#include <skills_arbitrator_msgs/SkillArbitration.h>

class SkillExecutionNode : public BT::SyncActionNode
{
public:
    SkillExecutionNode(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
      {
        return{BT::InputPort<std::string>("robot"),
               BT::InputPort<std::string>("action_name"),
               BT::InputPort<std::string>("skill_name")};
      }

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_exec_clnt_;
};

class ActionLearningNode : public BT::SyncActionNode
{
public:
    ActionLearningNode(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
      {
        return{BT::InputPort<std::string>("action_name"),
               BT::InputPort<std::string>("type")};
      }

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_learn_clnt_;
};

class ActionExploretionNode : public BT::SyncActionNode
{
public:
    ActionExploretionNode(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
      {
        return{BT::InputPort<std::string>("type"),
               BT::InputPort<std::string>("action_name")};
      }

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_explore_clnt_;
};

class ActionArbitrationNode : public BT::SyncActionNode
{
public:
    ActionArbitrationNode(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
      {
        return{BT::InputPort<std::string>("action_name")};
      }

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_arbit_clnt_;
};

#endif // SKILLS_CLASSES_H
