#include <skills_arbitrator/skills_arbit.h>

namespace skills_arbitrator
{

SkillsArbit::SkillsArbit(const ros::NodeHandle & n) : n_(n)
{
    if (!n_.getParam("/skills_arbitrator/skills_parameters_name_space", param_ns_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_arbitrator/skills_parameters_name_space param, defaul 'RL_params'");
        param_ns_ = "RL_params";
    }

    XmlRpc::XmlRpcValue action_types_to_evaluate;
    if (!n_.getParam("/skills_arbitrator/action_types_to_evaluate", action_types_to_evaluate))
    {
        ROS_ERROR_STREAM("No /skills_arbitrator/action_types_to_evaluate param");
        return;
    }

    std::vector<std::string> actions_types = skills_util::getMemberByXml(action_types_to_evaluate);

//    old
//    for (const std::string action_type: actions_types)
//    {
//        std::vector<std::string> index_names;
//        if (!n_.getParam("/skills_arbitrator/action_types_to_evaluate/" + action_type + "/index_names", index_names))
//        {
//            ROS_ERROR_STREAM("No /skills_arbitrator/action_types_to_evaluate/" + action_type + "/index_names param or it is not a vector of string");
//            return;
//        }
//        action_evaluation_indexes_.insert(std::make_pair(action_type,index_names));

//        std::vector<double> index_weights;
//        if (!n_.getParam("/skills_arbitrator/action_types_to_evaluate/" + action_type + "/index_weights", index_weights))
//        {
//            ROS_ERROR_STREAM("No /skills_arbitrator/action_types_to_evaluate/" + action_type + "/index_weights param or it is not a vector of double");
//            return;
//        }
//        action_evaluation_weights_.insert(std::make_pair(action_type,index_weights));
//    }
//    for (const std::pair<std::string,std::vector<std::string>> pair: action_evaluation_indexes_)
//    {
//        ROS_DEBUG_STREAM(pair.first + ":");
//        for (const std::string param: pair.second)
//        {
//            ROS_DEBUG_STREAM("  - " << param);
//        }
//    }

//    for (const std::pair<std::string,std::vector<double>> pair: action_evaluation_weights_)
//    {
//        ROS_DEBUG_STREAM(pair.first + ":");
//        for (const double param: pair.second)
//        {
//            ROS_DEBUG_STREAM("  - " << param);
//        }
//    }
//    end old

//    new
    for (const std::string action_type: actions_types)
    {
        XmlRpc::XmlRpcValue action_indexes;
        if (!n_.getParam("/skills_arbitrator/action_types_to_evaluate/" + action_type, action_indexes))
        {
            ROS_ERROR_STREAM("No /" + param_ns_ + "/" + action_type + " param");
            return;
        }

        std::vector<std::string> index_names = skills_util::getMemberByXml(action_indexes);
        double index_weight;
        std::map<std::string,double> action_info;
        for (std::string index_name: index_names)
        {
            if (!n_.getParam("/skills_arbitrator/action_types_to_evaluate/" + action_type + "/" + index_name, index_weight))
            {
                ROS_ERROR_STREAM("No /skills_arbitrator/action_types_to_evaluate/" + action_type + "/" + index_name + " param or it is not a vector of double");
                return;
            }
            action_info.insert(std::make_pair(index_name,index_weight));
        }
        actions_evaluation_info_.insert(std::make_pair(action_type,action_info));
    }

    ROS_INFO_STREAM("Skills_arbitrator params:");
    ROS_INFO_STREAM(" Actions:");
    for (const std::pair<std::string,std::map<std::string,double>> action_info: actions_evaluation_info_)
    {
        ROS_INFO_STREAM("  " + action_info.first);
        for (const std::pair<std::string,double> parameter_info: action_info.second)
            ROS_INFO_STREAM("   " + parameter_info.first + ". Weight: " + std::to_string(parameter_info.second));
    }
//    end new

    XmlRpc::XmlRpcValue skill_types_to_evaluate;
    if (!n_.getParam("/skills_arbitrator/skill_types_to_evaluate", skill_types_to_evaluate))
    {
        ROS_ERROR_STREAM("No /skills_arbitrator/skill_types_to_evaluate param");
        return;
    }

    std::vector<std::string> skill_types = skills_util::getMemberByXml(skill_types_to_evaluate);

//    old
//    for (const std::string skill_type: skills_types)
//    {
//        std::vector<std::string> index_names;
//        if (!n_.getParam("/skills_arbitrator/skill_types_to_evaluate/" + skill_type + "/index_names", index_names))
//        {
//            ROS_ERROR_STREAM("No /skills_arbitrator/skill_types_to_evaluate/" + skill_type + "/index_names param or it is not a vector of string");
//            return;
//        }
//        skill_evaluation_indexes_.insert(std::make_pair(skill_type,index_names));

//        std::vector<double> index_weights;
//        if (!n_.getParam("/skills_arbitrator/skill_types_to_evaluate/" + skill_type + "/index_weights", index_weights))
//        {
//            ROS_ERROR_STREAM("No /skills_arbitrator/skill_types_to_evaluate/" + skill_type + "/index_weights param or it is not a vector of double");
//            return;
//        }
//        skill_evaluation_weights_.insert(std::make_pair(skill_type,index_weights));
//    }

//    for (const std::pair<std::string,std::vector<std::string>> pair: skill_evaluation_indexes_)
//    {
//        ROS_DEBUG_STREAM(pair.first + ":");
//        for (const std::string param: pair.second)
//        {
//            ROS_DEBUG_STREAM("  - " << param);
//        }
//    }

//    for (const std::pair<std::string,std::vector<double>> pair: skill_evaluation_weights_)
//    {
//        ROS_DEBUG_STREAM(pair.first + ":");
//        for (const double param: pair.second)
//        {
//            ROS_DEBUG_STREAM("  - " << param);
//        }
//    }
//    end old

//    new
    for (const std::string skill_type: skill_types)
    {
        XmlRpc::XmlRpcValue skill_indexes;
        if (!n_.getParam("/skills_arbitrator/skill_types_to_evaluate/" + skill_type, skill_indexes))
        {
            ROS_ERROR_STREAM("/skills_arbitrator/skill_types_to_evaluate/" + skill_type + " param");
            return;
        }

        std::vector<std::string> index_names = skills_util::getMemberByXml(skill_indexes);
        double index_weight;
        std::map<std::string,double> skill_info;
        for (std::string index_name: index_names)
        {
            if (!n_.getParam("/skills_arbitrator/skill_types_to_evaluate/" + skill_type + "/" + index_name, index_weight))
            {
                ROS_ERROR_STREAM("No /skills_arbitrator/skill_types_to_evaluate/" + skill_type + "/" + index_name + " param or it is not a vector of double");
                return;
            }
            skill_info.insert(std::make_pair(index_name,index_weight));
        }

        skills_evaluation_info_.insert(std::make_pair(skill_type, skill_info));
    }

    ROS_INFO_STREAM(" Skills:");
    for (const std::pair<std::string,std::map<std::string,double>> skill_info: skills_evaluation_info_)
    {
        ROS_INFO_STREAM("  " + skill_info.first);
        for (const std::pair<std::string,double> parameter_info: skill_info.second)
            ROS_INFO_STREAM("   " + parameter_info.first + ". Weight: " + std::to_string(parameter_info.second));
    }
//    end new

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
        ROS_RED_STREAM("No param /"<<req.action_name<<"/executed, skill arbitration finish");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    if ( executed )
    {
        if (!getParam(req.action_name, "action_type", action_type))
        {
            ROS_RED_STREAM("No param /"<<req.action_name<<"/action_type, skill arbitration finish");
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
            return true;
        }
        ROS_WHITE_STREAM("Action type: "<<action_type.c_str());

//        old
//        if ( action_evaluation_indexes_.find(action_type) == action_evaluation_indexes_.end() )
//        {
//            ROS_RED_STREAM("/"<<action_type<<" action type is not in the list, skill arbitration finish");
//            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
//            return true;
//        }
//        end old

//        new
        if (actions_evaluation_info_.find(action_type) == actions_evaluation_info_.end() )
        {
            ROS_RED_STREAM("/"<<action_type<<" action type is not in the list, skill arbitration finish");
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
            return true;
        }
//        end new

        total_reward = 0.0;
        ROS_WHITE_STREAM("Start total reward: 0.0");

//        old
//        for (int i = 0; i < action_evaluation_indexes_[action_type].size(); i++)
//        {
//            double value;
//            if ( !getParam(req.action_name,action_evaluation_indexes_[action_type].at(i),value) )
//            {
//                ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<action_evaluation_indexes_[action_type].at(i));
//                setParam(req.action_name,action_evaluation_indexes_[action_type].at(i),0.0);
//                ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<action_evaluation_indexes_[action_type].at(i)<<": "<<0.0);
//                value = 0.0;
//            }
//            ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<action_evaluation_indexes_[action_type].at(i)<<": "<<value);
//            ROS_WHITE_STREAM("Total_reward + "<<value<<" * "<<action_evaluation_weights_[action_type].at(i));
//            total_reward = total_reward + ( value * action_evaluation_weights_[action_type].at(i) );
//            setParam(req.action_name,action_evaluation_indexes_[action_type].at(i)+"_reward",value * action_evaluation_weights_[action_type].at(i));
//        }
//        end old

//        new
        for (const auto& [index_name,index_weight]: actions_evaluation_info_[action_type])
        {
           double value;
            if ( !getParam(req.action_name,index_name,value) )
            {
                ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<index_name);
                setParam(req.action_name,index_name,0.0);
                ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<index_name<<": "<<0.0);
                value = 0.0;
            }
            ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<index_name<<": "<<value);
            ROS_WHITE_STREAM("Total_reward + " << value << " * " << index_weight);
            total_reward = total_reward + ( value * index_weight );
            setParam(req.action_name,index_name+"_reward",value * index_weight);
        }
//        end new

        setParam(req.action_name,"total_reward",total_reward);
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/total_reward: "<<total_reward);

//        old
//        for (int i = 0; i < action_evaluation_indexes_[action_type].size(); i++)
//        {
//            setParam(req.action_name,action_evaluation_indexes_[action_type].at(i), 0);
//            ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<action_evaluation_indexes_[action_type].at(i)<<": "<<0);
//        }
//        end old

//        new


//        for (std::pair<std::string,double> index_info: actions_evaluation_info_[action_type])
//        {
//            setParam(req.action_name,index_info.first, 0);
//            ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<index_info.first<<": "<<0);
//        }

        for (const auto& [index_name,index_weight]: actions_evaluation_info_[action_type])
        {
            setParam(req.action_name,index_name, 0);
            ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<index_name<<": "<<0);
        }

//        end new
    }
    else
    {
        ROS_RED_STREAM("/"<<req.action_name<<" not executed, skill arbitration finish");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Fail;
        return true;
    }

//    old
//    if (!n_.getParamNames(params))
//    {
//        ROS_RED_STREAM("Error with getParamNames");
//        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Error;
//        return true;
//    }

//    for ( const std::string param: params )
//    {
//        if ( param.find(req.action_name) != std::string::npos && param.find("skill_type") != std::string::npos )
//        {
//            std::string skill_name = param;
//            std::size_t index = skill_name.find("skill_type");
//            skill_name.erase(skill_name.begin()+index-1,skill_name.end());
//            index = skill_name.find_last_of("/");
//            skill_name.erase(skill_name.begin(),skill_name.begin()+index+1);
//            std::string skill_type;
//            if (!getParam(req.action_name, skill_name, "skill_type",   skill_type))
//            {
//                ROS_RED_STREAM("No param "<<req.action_name<<"/"<<skill_name<<"/skill_type, skill explore finish");
//                res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
//                return true;
//            }
//            skill_type_map.insert(std::make_pair(skill_name,skill_type));
//            skill_names.push_back(skill_name);
//        }
//    }

//    for ( const std::string skill_name: skill_names)
//    {
//        ROS_BOLDMAGENTA_STREAM("Skill name: "<<skill_name.c_str());

//        if (!getParam(req.action_name, skill_name, "executed",   executed))
//        {
//            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/executed");
//        }
//        else{
//            if ( executed )
//            {
//                if (!getParam(req.action_name, skill_name, "skill_type", skill_type_map.at(skill_name)))
//                {
//                    ROS_RED_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/skill_type, skill arbitration finish");
//                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
//                    return true;
//                }
//                ROS_WHITE_STREAM("Skill type: "<<skill_type_map.at(skill_name).c_str());

//                if ( skill_evaluation_indexes_.find(skill_type_map.at(skill_name)) == skill_evaluation_indexes_.end() )
//                {
//                    ROS_RED_STREAM("/"<<skill_type_map.at(skill_name)<<" skill type  is not in the list, skill arbitration finish");
//                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
//                    return true;
//                }

//                double reward = 0.0;

//                for (int i = 0; i < skill_evaluation_indexes_[skill_type_map.at(skill_name)].size(); i++)
//                {
//                    double value;
//                    if ( !getParam(req.action_name,skill_name,skill_evaluation_indexes_[skill_type_map.at(skill_name)].at(i),value) )
//                    {
//                        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_indexes_[skill_type_map.at(skill_name)].at(i) );
//                        setParam(req.action_name,skill_name,skill_evaluation_indexes_[skill_type_map.at(skill_name)].at(i),0.0);
//                        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_indexes_[skill_type_map.at(skill_name)].at(i)<<": "<<0.0);
//                        value = 0.0;
//                        setParam(req.action_name,skill_name,skill_evaluation_indexes_[skill_type_map.at(skill_name)].at(i),value);
//                    }
//                    ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_indexes_[skill_type_map.at(skill_name)].at(i)<<": "<<value);
//                    ROS_WHITE_STREAM("Reward + "<<value<<" * "<<skill_evaluation_weights_[skill_type_map.at(skill_name)].at(i));
//                    reward = reward + ( value * skill_evaluation_weights_[skill_type_map.at(skill_name)].at(i) );
//                }

//                setParam(req.action_name,skill_name,"reward",reward);
//                ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/reward: "<<reward);
//            }
//        }
//    }
//    end old

//    new
    XmlRpc::XmlRpcValue action_info;
    if (!n_.getParam("/" + param_ns_ + "/actions/" + req.action_name, action_info))
    {
        ROS_ERROR_STREAM("No /" + param_ns_ + "/actions/" + req.action_name + " param");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    std::vector<std::string> action_members = skills_util::getMemberByXml(action_info);
    for (const auto& action_member: action_members)
        if (n_.hasParam("/"+param_ns_+"/"+req.action_name+"/"+action_member+"/skill_type"))
            skill_names.push_back(action_member);

    for (const std::string skill_name: skill_names)
    {
        ROS_BOLDMAGENTA_STREAM("Skill name: "<<skill_name.c_str());

        if (!getParam(req.action_name, skill_name, "executed",   executed))
        {
            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/executed");
        }
        else{
            if ( executed )
            {
                std::string skill_type;
                if (!getParam(req.action_name, skill_name, "skill_type", skill_type))
                {
                    ROS_RED_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/skill_type, skill arbitration finish");
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
                    return true;
                }
                ROS_WHITE_STREAM("Skill type: "<<skill_type.c_str());

                if (skills_evaluation_info_.find(skill_type) == skills_evaluation_info_.end())
                {
                    ROS_RED_STREAM("/"<<skill_type<<" skill type  is not in the list, skill arbitration finish");
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
                    return true;
                }

                double reward = 0.0;

                for (const auto& [index_name,index_weight]: actions_evaluation_info_[action_type])
                {
                    double value;

                    if (!getParam(req.action_name,skill_name,index_name,value))
                    {
                        ROS_YELLOW_STREAM("No param /" << req.action_name << "/" << skill_name << "/" << index_name);
                        setParam(req.action_name,skill_name,index_name,0.0);
                        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/"<<index_name<<": "<<0.0);
                        value = 0.0;
                        setParam(req.action_name,skill_name,index_name,value);
                    }
                    ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<skill_name<<"/"<<index_name<<": "<<value);
                    ROS_WHITE_STREAM("Reward + "<<value<<" * "<<index_weight);
                    reward = reward + ( value * index_weight );
                }
                setParam(req.action_name,skill_name,"reward",reward);
                ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/reward: "<<reward);
            }
        }
    }
//    end new

    ROS_BOLDMAGENTA_STREAM("Arbitrator has finished.");
    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Success;
    return true;
}

} // end namespace skills_arbitrator
