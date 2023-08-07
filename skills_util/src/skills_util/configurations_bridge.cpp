#include <skills_util/configurations_bridge.h>

namespace skills_util
{

ConfigurationBridge::ConfigurationBridge(const ros::NodeHandle &n): n_(n)
{
    start_config_clnt_ = n_.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
    ROS_WARN_STREAM("Waiting for "<<start_config_clnt_.getService());
    start_config_clnt_.waitForExistence();
    ROS_WARN_STREAM("Connection ok");

    list_config_clnt_ = n_.serviceClient<configuration_msgs::ListConfigurations>("/configuration_manager/list_configurations");
    ROS_WARN_STREAM("Waiting for "<<list_config_clnt_.getService());
    list_config_clnt_.waitForExistence();
    ROS_WARN_STREAM("Connection ok");

    std::vector<std::string> robots;
    if (!n_.getParam("/skills_executer/robots", robots))
    {
        ROS_ERROR_STREAM("No /skill_executer/robots param");
        exit(0);
    }

    for(const std::string robot: robots)
    {
        robots_configs.insert(std::make_pair(robot,""));
    }

    change_config_srv_ = n_.advertiseService("/skills_util/change_config", &ConfigurationBridge::changeConfig, this);

    ROS_INFO_STREAM("Ready to change robot config");
}


bool ConfigurationBridge::changeConfig(skills_util_msgs::ChangeConfig::Request  &req,
                                       skills_util_msgs::ChangeConfig::Response &res)
{
    configuration_msgs::ListConfigurations configs_list;

    if (!list_config_clnt_.call(configs_list))
    {
        ROS_ERROR("Unable to call %s service",list_config_clnt_.getService().c_str());
        return false;
    }

    std::string current_config;
    for (const configuration_msgs::ConfigurationComponent config : configs_list.response.configurations)
    {
        if (!config.state.compare("running"))
        {
            current_config = config.name;
        }
    }

    if (current_config.empty())
    {
        ROS_ERROR_STREAM("Running config not found");
        return false;
    }

    std::size_t found = current_config.find(req.robot_name);

    std::string current_robot_config;

    if ( found != std::string::npos )
    {
        current_robot_config = current_config.substr(found);
        found = current_robot_config.find("__");
        if ( found != std::string::npos )
        {
            current_robot_config = current_robot_config.substr(0,found);
        }
        current_robot_config.replace(0,req.robot_name.length()+1, "");
    }
    else
    {
        ROS_ERROR("Robot name not in current config name");
        return false;
    }

    std::string new_config = current_config;
    found = new_config.find(req.robot_name + "_" + current_robot_config);
    new_config.replace(found + req.robot_name.length() + 1, current_robot_config.length(), req.config_name);

    configuration_msgs::StartConfiguration start_config_srv;
    start_config_srv.request.start_configuration = new_config;
    start_config_srv.request.strictness = 1;

    if (!start_config_clnt_.call(start_config_srv))
    {
      ROS_ERROR("Unable to call %s service to set controller %s",start_config_clnt_.getService().c_str(),req.config_name.c_str());
      return false;
    }

    res.ok = start_config_srv.response.ok;

    if (!start_config_srv.response.ok)
    {
      ROS_ERROR("Error on service %s response", start_config_clnt_.getService().c_str());
      return false;
    }

    ROS_INFO_STREAM("Controller "<<new_config<<" started.");

    ros::Duration(0.1).sleep();

    return true;
}

} // end namespace skills_util
