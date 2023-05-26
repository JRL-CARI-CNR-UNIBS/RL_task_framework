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
    std::string repo = req.folder_path;
    repo.append("/");
    for ( auto const& entry : std::filesystem::directory_iterator(repo))
    {
        if( entry.path().extension() == ".xml")
        {
            ROS_INFO_STREAM("Repo: "<<entry.path().string());
            factory_.registerBehaviorTreeFromFile(entry.path().string());
        }
    }

    BT::Tree tree = factory_.createTree(req.tree_name);

    BT::NodeStatus result;
    result = tree.tickWhileRunning();
    res.result =  int( result );

    factory_.clearRegisteredBehaviorTrees();

    ROS_INFO("BT success");
    return true;
}


} // end namespace skills_executer
