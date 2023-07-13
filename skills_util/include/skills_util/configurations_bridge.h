#ifndef CONFIGURATIONS_BRIDGE_H
#define CONFIGURATIONS_BRIDGE_H

#include <ros/ros.h>
#include <skills_util_msgs/ChangeConfig.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/ListConfigurations.h>
#include <configuration_msgs/ConfigurationComponent.h>

namespace skills_util
{

class ConfigurationBridge
{
public:
    ConfigurationBridge(const ros::NodeHandle &n);

    bool changeConfig(skills_util_msgs::ChangeConfig::Request  &req,
                      skills_util_msgs::ChangeConfig::Response &res);

private:
    ros::NodeHandle n_;

    ros::ServiceServer change_config_srv_;
    ros::ServiceClient start_config_clnt_;
    ros::ServiceClient list_config_clnt_;

    std::map<std::string,std::string> robots_configs;
};
} // end namespace skills_util


#endif // CONFIGURATIONS_BRIDGE_H
