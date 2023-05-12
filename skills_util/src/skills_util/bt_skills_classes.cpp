#include <skills_util/bt_skills_classes.h>

std::vector<std::string> bt_skills_classes::skillNames(const std::string& action_skill_name)
{
    std::size_t found = action_skill_name.find("/");
    std::vector<std::string> names;
    if (found==std::string::npos)
    {
        ROS_WARN("The name of action don't respect the standard: %s", action_skill_name.c_str());
        return names;
    }
    std::string action_name = action_skill_name;
    action_name.erase(action_name.begin()+found, action_name.end());
    std::string skill_name = action_skill_name;
    skill_name.erase(skill_name.begin(), skill_name.begin()+found+1);

    ROS_INFO("action_skill_name: %s",action_skill_name.c_str());
    ROS_INFO("action_name: %s",action_name.c_str());
    ROS_INFO("skill_name: %s",skill_name.c_str());

    names.push_back(action_name);
    names.push_back(skill_name);
    return names;
}

SkillActionNode::SkillActionNode(const std::string &name) : BT::SyncActionNode(name, {})
{
    ROS_INFO("Init of SkillActionNode, action name: %s", name.c_str());

    skill_exec_clnt_ = n_.serviceClient<skills_executer_msgs::SkillExecution>("/skills_exec/execute_skill");
    ROS_WARN("Waiting for %s", skill_exec_clnt_.getService().c_str() );
    skill_exec_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus SkillActionNode::tick()
{
    std::vector<std::string> names = bt_skills_classes::skillNames(name());
    ROS_INFO("action_name: %s",names.at(0).c_str());
    ROS_INFO("skill_name: %s",names.at(1).c_str());
    if (names.empty())
    {
        ROS_WARN("The name of action don't respect the standard");
        return BT::NodeStatus::FAILURE;
    }
    skills_executer_msgs::SkillExecution skill_exec_srv;
    skill_exec_srv.request.action_name = names.at(0);
    skill_exec_srv.request.skill_name = names.at(1);
//        mettere qua tutto il necessario

    if (!skill_exec_clnt_.call(skill_exec_srv))
    {
        ROS_ERROR("Fail to call service: %s", skill_exec_clnt_.getService().c_str());
        ROS_ERROR("action_name: %s, skill_name: %s", skill_exec_srv.request.action_name.c_str(), skill_exec_srv.request.skill_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

SkillExecutionNode::SkillExecutionNode(const std::string &name) : BT::SyncActionNode(name,  {})
{
    ROS_INFO("Init of SkillExecutionNode, action name: %s", name.c_str());

    skill_exec_clnt_ = n_.serviceClient<skills_executer_msgs::SkillExecution>("/skills_exec/execute_skill");
    ROS_WARN("Waiting for %s", skill_exec_clnt_.getService().c_str() );
    skill_exec_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus SkillExecutionNode::tick()
{
    std::vector<std::string> names = bt_skills_classes::skillNames(name());
    ROS_INFO("action_name: %s",names.at(0).c_str());
    ROS_INFO("skill_name: %s",names.at(1).c_str());
    if (names.empty())
    {
        ROS_WARN("The name of action don't respect the standard");
        return BT::NodeStatus::FAILURE;
    }
    skills_executer_msgs::SkillExecution skill_exec_srv;
    skill_exec_srv.request.action_name = names.at(0);
    skill_exec_srv.request.skill_name = names.at(1);
//        mettere qua tutto il necessario

    if (!skill_exec_clnt_.call(skill_exec_srv))
    {
        ROS_ERROR("Fail to call service: %s", skill_exec_clnt_.getService().c_str());
        ROS_ERROR("action_name: %s, skill_name: %s", skill_exec_srv.request.action_name.c_str(), skill_exec_srv.request.skill_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if ( skill_exec_srv.response.result < 0 )
    {
        ROS_WARN("/%s/%s has failed", skill_exec_srv.request.action_name.c_str(), skill_exec_srv.request.skill_name.c_str());
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
    ROS_INFO("action_name: %s",name().c_str());
    if (name().empty())
    {
        ROS_WARN("No action name");
        return BT::NodeStatus::FAILURE;
    }

    std::string learning_type;
    if(!ActionLearningNode::getInput<std::string>("type",learning_type))
      {
        ROS_ERROR("Missing required input [type]");
        throw BT::RuntimeError("Missing required input [type]");
      }

    skills_learning_msgs::SkillLearning skill_learn_srv;
    skill_learn_srv.request.action_name = name();
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
    ROS_INFO("action_name: %s",name().c_str());
    if (name().empty())
    {
        ROS_WARN("No action name");
        return BT::NodeStatus::FAILURE;
    }

    std::string exploration_type;
    if(!ActionExploretionNode::getInput<std::string>("type",exploration_type))
      {
        ROS_ERROR("Missing required input [type]");
        throw BT::RuntimeError("Missing required input [type]");
      }

    skills_learning_msgs::SkillExplore skill_explore_srv;
    skill_explore_srv.request.action_name = name();
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

ActionArbitrationNode::ActionArbitrationNode(const std::string &name) : BT::SyncActionNode(name, {})
{
    ROS_INFO("Init of ActionLearningNode, action name: %s", name.c_str());

    skill_arbit_clnt_ = n_.serviceClient<skills_arbitrator_msgs::SkillArbitration>("/skills_arbit/evaluate_skill");
    ROS_WARN("Waiting for %s", skill_arbit_clnt_.getService().c_str() );
    skill_arbit_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus ActionArbitrationNode::tick()
{
    ROS_INFO("action_name: %s",name().c_str());
    if (name().empty())
    {
        ROS_WARN("No action name");
        return BT::NodeStatus::FAILURE;
    }

    skills_arbitrator_msgs::SkillArbitration skill_arbit_srv;
    skill_arbit_srv.request.action_name = name();
    if (!skill_arbit_clnt_.call(skill_arbit_srv))
    {
        ROS_ERROR("Fail to call service: %s", skill_arbit_clnt_.getService().c_str());
        ROS_ERROR("action_name: %s", skill_arbit_srv.request.action_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}
