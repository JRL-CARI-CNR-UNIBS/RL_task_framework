#include <skills_util/bt_skills_classes.h>

SkillExecutionNode::SkillExecutionNode(const std::string &name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
    ROS_INFO_STREAM("Init of SkillExecutionNode");

    if ( !ros::service::exists("/skills_exec/execute_skill", false) )
    {
        throw std::runtime_error("failed to construct");;
    }

    skill_exec_clnt_ = n_.serviceClient<skills_executer_msgs::RobotSkillExecution>("/skills_exec/execute_skill");
    ROS_WARN("Waiting for %s", skill_exec_clnt_.getService().c_str() );
    skill_exec_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus SkillExecutionNode::tick()
{
    std::string skill_name;
    if (!SkillExecutionNode::getInput<std::string>("skill_name",skill_name))
    {
        ROS_ERROR("Missing required input skill_name");
        return BT::NodeStatus::FAILURE;
    }
    std::string action_name;
    if (!SkillExecutionNode::getInput<std::string>("action_name",action_name))
    {
        ROS_ERROR("Missing required input action_name");
        return BT::NodeStatus::FAILURE;
    }

    std::string robot_name;
    if (!SkillExecutionNode::getInput<std::string>("robot",robot_name))
    {
        ROS_ERROR("Missing required input robot");
        return BT::NodeStatus::FAILURE;
    }

    skills_executer_msgs::RobotSkillExecution skill_exec_srv;
    skill_exec_srv.request.action_name = action_name;
    skill_exec_srv.request.skill_name = skill_name;
    skill_exec_srv.request.robot_name = robot_name;

    if (!skill_exec_clnt_.call(skill_exec_srv))
    {
        ROS_ERROR("Success: false");
        ROS_ERROR("Fail to call service: %s", skill_exec_clnt_.getService().c_str());
        ROS_ERROR("action_name: %s, skill_name: %s", skill_exec_srv.request.action_name.c_str(), skill_exec_srv.request.skill_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if ( skill_exec_srv.response.result == -1 )
    {
        ROS_WARN("/%s/%s has failed", skill_exec_srv.request.action_name.c_str(), skill_exec_srv.request.skill_name.c_str());
        return BT::NodeStatus::FAILURE;
    }
    else if ( skill_exec_srv.response.result < -1 )
    {
        ROS_ERROR("Error with /%s/%s", skill_exec_srv.request.action_name.c_str(), skill_exec_srv.request.skill_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

ActionLearningNode::ActionLearningNode(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config)
{
    ROS_INFO("Init of ActionLearningNode, action name: %s", name.c_str());

    skill_learn_clnt_ = n_.serviceClient<skills_learning_msgs::SkillLearning>("/skills_learn/learn_skill");
    ROS_WARN("Waiting for %s", skill_learn_clnt_.getService().c_str() );
    skill_learn_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus ActionLearningNode::tick()
{
    std::string action_name;
    if (!ActionLearningNode::getInput<std::string>("action_name",action_name))
    {
        ROS_ERROR("Missing required input action_name");
        return BT::NodeStatus::FAILURE;
    }

    std::string learning_type;
    if(!ActionLearningNode::getInput<std::string>("type",learning_type))
      {
        ROS_ERROR("Missing required input [type]");
        return BT::NodeStatus::FAILURE;
      }

    skills_learning_msgs::SkillLearning skill_learn_srv;
    skill_learn_srv.request.action_name = action_name;
    skill_learn_srv.request.learning_type = learning_type;
    if (!skill_learn_clnt_.call(skill_learn_srv))
    {
        ROS_ERROR("Fail to call service: %s", skill_learn_clnt_.getService().c_str());
        ROS_ERROR("action_name: %s", skill_learn_srv.request.action_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

ActionExploretionNode::ActionExploretionNode(const std::string &name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
    ROS_INFO("Init of ActionExploreNode, action name: %s", name.c_str());

    skill_explore_clnt_ = n_.serviceClient<skills_learning_msgs::SkillExplore>("/skills_learn/explore_skill");
    ROS_WARN("Waiting for %s", skill_explore_clnt_.getService().c_str() );
    skill_explore_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus ActionExploretionNode::tick()
{
    std::string action_name;
    if (!ActionExploretionNode::getInput<std::string>("action_name",action_name))
    {
        ROS_ERROR("Missing required input action_name");
        return BT::NodeStatus::FAILURE;
    }

    std::string exploration_type;
    if(!ActionExploretionNode::getInput<std::string>("type",exploration_type))
      {
        ROS_ERROR("Missing required input [type]");
        return BT::NodeStatus::FAILURE;
      }

    skills_learning_msgs::SkillExplore skill_explore_srv;
    skill_explore_srv.request.action_name = action_name;
    skill_explore_srv.request.exploration_type = exploration_type;
    if (!skill_explore_clnt_.call(skill_explore_srv))
    {
        ROS_ERROR("Fail to call service: %s", skill_explore_clnt_.getService().c_str());
        ROS_ERROR("Result: %d", skill_explore_srv.response.result);
        ROS_ERROR("action_name: %s", skill_explore_srv.request.action_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

ActionArbitrationNode::ActionArbitrationNode(const std::string &name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
{
    ROS_INFO("Init of ActionLearningNode, action name: %s", name.c_str());

    skill_arbit_clnt_ = n_.serviceClient<skills_arbitrator_msgs::SkillArbitration>("/skills_arbit/evaluate_skill");
    ROS_WARN("Waiting for %s", skill_arbit_clnt_.getService().c_str() );
    skill_arbit_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus ActionArbitrationNode::tick()
{
    std::string action_name;
    if (!ActionArbitrationNode::getInput<std::string>("action_name",action_name))
    {
        ROS_ERROR("Missing required input action_name");
        return BT::NodeStatus::FAILURE;
    }

    skills_arbitrator_msgs::SkillArbitration skill_arbit_srv;
    skill_arbit_srv.request.action_name = action_name;
    if (!skill_arbit_clnt_.call(skill_arbit_srv))
    {
        ROS_ERROR("Fail to call service: %s", skill_arbit_clnt_.getService().c_str());
        ROS_ERROR("action_name: %s", skill_arbit_srv.request.action_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}
