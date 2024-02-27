#include <skills_arbitrator/skills_arbit.h>

namespace skills_arbitrator
{

SkillsArbit::SkillsArbit(const ros::NodeHandle & n) : n_(n)
{
    if (!n_.getParam("/skills_executer/skills_parameters_name_space", exec_param_ns_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skills_parameters_name_space param, defaul 'exec_params'");
        exec_param_ns_ = "exec_params";
    }

    if (!n_.getParam("/skills_arbitrator/arbitrator_parameters_name_space", arbit_param_ns_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_arbitrator/arbitrator_parameters_name_space param, defaul 'arbit_params'");
        arbit_param_ns_ = "arbit_params";
    }

    XmlRpc::XmlRpcValue actions_to_evaluate;
    if (!n_.getParam("/" + arbit_param_ns_ + "/actions", actions_to_evaluate))
    {
        ROS_ERROR_STREAM("No /skills_arbitrator/action_types_to_evaluate param");
        return;
    }

    std::vector<std::string> action_names = skills_util::getMemberByXml(actions_to_evaluate);

    for (const std::string action_name: action_names)
    {
        XmlRpc::XmlRpcValue action_params;
        if (!n_.getParam("/" + arbit_param_ns_ + "/actions/" + action_name, action_params))
        {
            ROS_ERROR_STREAM("No /" + arbit_param_ns_ + "/actions/" + action_name + " param");
            return;
        }
        std::vector<std::string> param_names = skills_util::getMemberByXml(action_params);
        std::map<std::string,double> param_name_to_param_value;
        for (const auto param_name: param_names)
        {
            if (!param_name.compare("skills"))
                continue;

            double weight;
            XmlRpc::XmlRpcValue param_xml;
            if (!n_.getParam("/" + arbit_param_ns_ + "/actions/" + action_name + "/" + param_name, param_xml))
            {
                ROS_ERROR_STREAM("No /" + arbit_param_ns_ + "/actions/" + action_name + "/" + param_name + " param");
                return;
            }
            else
            {
                if (param_xml.getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
                {
                    weight = double(param_xml);
                }
                else if (param_xml.getType() == XmlRpc::XmlRpcValue::Type::TypeInt)
                {
                    weight = double(int(param_xml));
                }
                else
                {
                    ROS_ERROR_STREAM("/" + arbit_param_ns_ + "/actions/" + action_name + "/" + param_name + " value type is wrong.");
                    return;
                }
            }
            param_name_to_param_value.insert(std::make_pair(param_name,weight));
        }
        action_evaluation_parameters_.insert(std::make_pair(action_name,param_name_to_param_value));

        XmlRpc::XmlRpcValue skills_param;
        if (!n_.getParam("/" + arbit_param_ns_ + "/actions/" + action_name + "/skills", skills_param))
        {
            ROS_ERROR_STREAM("No /" + arbit_param_ns_ + "/actions/" + action_name + "/skills param");
            return;
        }
        std::vector<std::string> skill_names = skills_util::getMemberByXml(skills_param);
        std::map<std::string,std::map<std::string,double>> skill_name_to_params;
        for (const std::string skill_name: skill_names)
        {
            XmlRpc::XmlRpcValue params;
            if (!n_.getParam("/" + arbit_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name, params))
            {
                ROS_ERROR_STREAM("No /" + arbit_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + " param");
                return;
            }
            std::vector<std::string> param_names = skills_util::getMemberByXml(params);
            std::map<std::string,double> param_name_to_param_value;
            for (const std::string param_name: param_names)
            {
                double weight;
                XmlRpc::XmlRpcValue param_xml;
                if (!n_.getParam("/" + arbit_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + "/" + param_name, param_xml))
                {
                    ROS_ERROR_STREAM("No /" + arbit_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + "/" + param_name + " param");
                    return;
                }
                else
                {
                    if (param_xml.getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
                    {
                        weight = double(param_xml);
                    }
                    else if (param_xml.getType() == XmlRpc::XmlRpcValue::Type::TypeInt)
                    {
                        weight = double(int(param_xml));
                    }
                    else
                    {
                        ROS_ERROR_STREAM("/" + arbit_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + "/" + param_name + " value type is wrong.");
                        return;
                    }
                }
                param_name_to_param_value.insert(std::make_pair(param_name,weight));
            }
            skill_name_to_params.insert(std::make_pair(skill_name,param_name_to_param_value));
        }
        skill_evaluation_parameters_.insert(std::make_pair(action_name,skill_name_to_params));
    }

    ROS_INFO_STREAM(arbit_param_ns_ + " params:");
    for (const auto action: skill_evaluation_parameters_)
    {
         ROS_INFO_STREAM("  " + action.first + ":");
         for (const auto param: action_evaluation_parameters_[action.first])
         {
             ROS_INFO_STREAM("    " +param.first+": " + std::to_string(param.second));
         }
         for (const auto skill: action.second)
         {
             ROS_INFO_STREAM("    " +skill.first+":");
             for (const auto param: skill.second)
             {
                 ROS_INFO_STREAM("      " + param.first + ": " + std::to_string(param.second));
             }
         }
    }

    skill_arbit_srv_ = n_.advertiseService("/skills_arbit/evaluate_skill", &SkillsArbit::skillsArbitration, this);
}

bool SkillsArbit::skillsArbitration(skills_arbitrator_msgs::SkillArbitration::Request  &req,
                                    skills_arbitrator_msgs::SkillArbitration::Response &res)
{
    ROS_BOLDMAGENTA_STREAM("Action to evaluate: "<<req.action_name.c_str());
    std::string action_type;
    std::vector<std::string> skill_names, params;
    std::map<std::string,std::string> skill_type_map;

    double total_reward = 0.0;
    int executed;

    if (!getParam(req.action_name, "executed", executed))
    {
        ROS_ERROR_STREAM("No param /"<<req.action_name<<"/executed, skill arbitration finish");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    if ( executed )
    {
        total_reward = 0.0;
        ROS_INFO_STREAM("Start total reward: 0.0");
        bool fail = false;
        for (const auto param: action_evaluation_parameters_[req.action_name])
        {
            double value;
            if (!getParam(req.action_name,param.first,value))
            {
                ROS_ERROR_STREAM("No param /"<<req.action_name<<"/"<<param.first);
                fail = true;
                break;
            }
            ROS_INFO_STREAM("/"<<req.action_name<<"/"<<param.first<<": "<<value);
            ROS_INFO_STREAM("Total_reward + " << value << " * " << param.second);
            total_reward = total_reward + ( value * param.second );
            setParam(req.action_name,param.first+"_reward",value * param.second);
        }
        if (!fail)
        {
            setParam(req.action_name,"total_reward",total_reward);
            ROS_INFO_STREAM("Set /"<<req.action_name<<"/total_reward: "<<total_reward);
        }
        for (const auto param: action_evaluation_parameters_[req.action_name])
        {
            setParam(req.action_name,param.first, 0);
            ROS_INFO_STREAM("Set /"<<req.action_name<<"/"<<param.first<<": "<<0);
        }
    }
    else
    {
        ROS_ERROR_STREAM("/"<<req.action_name<<" not executed, skill arbitration finish");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Fail;
        return true;
    }

    XmlRpc::XmlRpcValue action_info;
    if (!n_.getParam("/" + exec_param_ns_ + "/actions/" + req.action_name, action_info))
    {
        ROS_ERROR_STREAM("No /" + exec_param_ns_ + "/actions/" + req.action_name + " param");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    for (const auto skill_params: skill_evaluation_parameters_[req.action_name])
    {
        ROS_BOLDMAGENTA_STREAM("Skill name: "<<skill_params.first.c_str());

        if (!getParam(req.action_name, skill_params.first, "executed",   executed))
        {
            ROS_WARN_STREAM("No param /"<<req.action_name<<"/"<<skill_params.first<<"/executed");
            continue;
        }
        else{
            if ( executed )
            {
                double reward = 0.0;
                bool fail = false;
                for (const auto param: skill_params.second)
                {
                    double value;
                    if (!getParam(req.action_name,skill_params.first,param.first,value))
                    {
                        ROS_ERROR_STREAM("No param /" << req.action_name << "/" << skill_params.first << "/" << param.first);
                        fail = true;
                        break;
                    }
                    ROS_INFO_STREAM("/"<<req.action_name<<"/"<<skill_params.first<<"/"<<param.first<<": "<<value);
                    ROS_INFO_STREAM("Reward + "<<value<<" * "<<param.second);
                    reward = reward + ( value * param.second );
                }
                if (fail)
                    break;
                setParam(req.action_name,skill_params.first,"reward",reward);
                ROS_INFO_STREAM("Set /"<<req.action_name<<"/"<<skill_params.first<<"/reward: "<<reward);
            }
        }
    }

    ROS_BOLDMAGENTA_STREAM("Arbitrator has finished.");
    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Success;
    return true;
}

} // end namespace skills_arbitrator
