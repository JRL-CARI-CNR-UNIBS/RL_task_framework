#include <skills_util/bt_exec.h>

namespace skills_util
{

BTExec::BTExec(const ros::NodeHandle & n) : n_(n)
{
    factory_.registerNodeType<SkillExecutionNode>("SkillExecutionNode");
    ROS_INFO("SkillExecutionNode registered");

    factory_.registerNodeType<ActionLearningNode>("ActionLearningNode");
    ROS_INFO("ActionLearningNode registered");

    factory_.registerNodeType<ActionExploretionNode>("ActionExploretionNode");
    ROS_INFO("ActionExploretionNode registered");

    factory_.registerNodeType<ActionArbitrationNode>("ActionArbitrationNode");
    ROS_INFO("ActionArbitrationNode registered");

    bt_exec_srv_ = n_.advertiseService("/skills_util/run_tree", &BTExec::runTree, this);
}

bool BTExec::runTree(skills_util_msgs::RunTree::Request  &req,
                     skills_util_msgs::RunTree::Response &res)
{
    std::vector<std::string> repos = req.folders_paths;
    ROS_INFO_STREAM("Repos size: "<<repos.size());

    for (std::string repo: repos)
    {
        ROS_INFO_STREAM("Repository: "<<repo);
        repo.append("/");
        for ( auto const& entry : std::filesystem::directory_iterator(repo))
        {
//            ROS_INFO_STREAM("  File: "<<entry.path().string());
            if( entry.path().extension() == ".xml")
            {
                ROS_INFO_STREAM("  File: "<<entry.path().string());
                factory_.registerBehaviorTreeFromFile(entry.path().string());
            }
        }
    }

    ROS_INFO_STREAM("Tree_name: "<<req.tree_name);

    BT::Tree tree = factory_.createTree(req.tree_name);

    BT::NodeStatus result;
    result = tree.tickWhileRunning();
    res.result =  int( result );

    factory_.clearRegisteredBehaviorTrees();

    ROS_INFO("BT finished");
    return true;
}


} // end namespace skills_executer
