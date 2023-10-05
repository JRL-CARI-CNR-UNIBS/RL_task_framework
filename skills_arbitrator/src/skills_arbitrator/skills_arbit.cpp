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

    XmlRpc::XmlRpcValue actions_evaluation_parameters;
    if (!n_.getParam("/skills_arbitrator/actions_evaluation_parameters", actions_evaluation_parameters))
    {
        ROS_ERROR_STREAM("No /skills_arbitrator/actions_evaluation_parameters param");
        return;
    }

    std::vector<std::string> actions_types = skills_util::getMemberByXml(actions_evaluation_parameters);

    for (const std::string action_type: actions_types)
    {
        std::vector<std::string> parameters_names;
        if (!n_.getParam("/skills_arbitrator/actions_evaluation_parameters/" + action_type, parameters_names))
        {
            ROS_ERROR_STREAM("No /skills_arbitrator/actions_evaluation_parameters/" + action_type + " param or it is not a vector of string");
            return;
        }
        actions_evaluation_parameters_.insert(std::make_pair(action_type,parameters_names));

        std::vector<double> parameters_weights;
        if (!n_.getParam("/skills_arbitrator/actions_parameters_weights/" + action_type, parameters_weights))
        {
            ROS_ERROR_STREAM("No /skills_arbitrator/actions_parameters_weights/" + action_type + " param or it is not a vector of double");
            return;
        }
        actions_evaluation_weights_.insert(std::make_pair(action_type,parameters_weights));
    }

//    for (const std::pair<std::string,std::vector<std::string>> pair: actions_evaluation_parameters_)
//    {
//        ROS_WARN_STREAM(pair.first + ":");
//        for (const std::string param: pair.second)
//        {
//            ROS_WARN_STREAM("  - " << param);
//        }
//    }

//    for (const std::pair<std::string,std::vector<double>> pair: actions_evaluation_weights_)
//    {
//        ROS_WARN_STREAM(pair.first + ":");
//        for (const double param: pair.second)
//        {
//            ROS_WARN_STREAM("  - " << param);
//        }
//    }

    XmlRpc::XmlRpcValue skills_evaluation_parameters;
    if (!n_.getParam("/skills_arbitrator/skills_evaluation_parameters", skills_evaluation_parameters))
    {
        ROS_ERROR_STREAM("No /skills_arbitrator/skills_evaluation_parameters param");
        return;
    }

    std::vector<std::string> skills_types = skills_util::getMemberByXml(skills_evaluation_parameters);

    for (const std::string skill_type: skills_types)
    {
        std::vector<std::string> parameters_names;
        if (!n_.getParam("/skills_arbitrator/skills_evaluation_parameters/" + skill_type, parameters_names))
        {
            ROS_ERROR_STREAM("No /skills_arbitrator/skills_evaluation_parameters/" + skill_type + " param or it is not a vector of string");
            return;
        }
        skills_evaluation_parameters_.insert(std::make_pair(skill_type,parameters_names));

        std::vector<double> parameters_weights;
        if (!n_.getParam("/skills_arbitrator/skills_parameters_weights/" + skill_type, parameters_weights))
        {
            ROS_ERROR_STREAM("No /skills_arbitrator/skills_parameters_weights/" + skill_type + " param or it is not a vector of double");
            return;
        }
        skills_evaluation_weights_.insert(std::make_pair(skill_type,parameters_weights));
    }

//    for (const std::pair<std::string,std::vector<std::string>> pair: skills_evaluation_parameters_)
//    {
//        ROS_WARN_STREAM(pair.first + ":");
//        for (const std::string param: pair.second)
//        {
//            ROS_WARN_STREAM("  - " << param);
//        }
//    }

//    for (const std::pair<std::string,std::vector<double>> pair: skills_evaluation_weights_)
//    {
//        ROS_WARN_STREAM(pair.first + ":");
//        for (const double param: pair.second)
//        {
//            ROS_WARN_STREAM("  - " << param);
//        }
//    }

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

        if ( actions_evaluation_parameters_.find(action_type) == actions_evaluation_parameters_.end() )
        {
            ROS_RED_STREAM("/"<<action_type<<" action type is not in the list, skill arbitration finish");
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
            return true;
        }

        total_reward = 0.0;
        ROS_WHITE_STREAM("Start total reward: 0.0");

        for (int i = 0; i < actions_evaluation_parameters_[action_type].size(); i++)
        {
            double value;
            if ( !getParam(req.action_name,actions_evaluation_parameters_[action_type].at(i),value) )
            {
                ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<actions_evaluation_parameters_[action_type].at(i));
                setParam(req.action_name,actions_evaluation_parameters_[action_type].at(i),0.0);
                ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<actions_evaluation_parameters_[action_type].at(i)<<": "<<0.0);
                value = 0.0;
            }
            ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<actions_evaluation_parameters_[action_type].at(i)<<": "<<value);
            ROS_WHITE_STREAM("Total_reward + "<<value<<" * "<<actions_evaluation_weights_[action_type].at(i));
            total_reward = total_reward + ( value * actions_evaluation_weights_[action_type].at(i) );
            setParam(req.action_name,actions_evaluation_parameters_[action_type].at(i)+"_reward",value * actions_evaluation_weights_[action_type].at(i));
        }

        setParam(req.action_name,"total_reward",total_reward);
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/total_reward: "<<total_reward);

        for (int i = 0; i < actions_evaluation_parameters_[action_type].size(); i++)
        {
            setParam(req.action_name,actions_evaluation_parameters_[action_type].at(i), 0);
            ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<actions_evaluation_parameters_[action_type].at(i)<<": "<<0);
        }
    }
    else
    {
        ROS_RED_STREAM("/"<<req.action_name<<" not executed, skill arbitration finish");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Fail;
        return true;
    }

    if (!n_.getParamNames(params))
    {
        ROS_RED_STREAM("Error with getParamNames");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Error;
        return true;
    }

    for ( const std::string param: params )
    {
        if ( param.find(req.action_name) != std::string::npos && param.find("skill_type") != std::string::npos )
        {
            std::string skill_name = param;
            std::size_t index = skill_name.find("skill_type");
            skill_name.erase(skill_name.begin()+index-1,skill_name.end());
            index = skill_name.find_last_of("/");
            skill_name.erase(skill_name.begin(),skill_name.begin()+index+1);
            std::string skill_type;
            if (!getParam(req.action_name, skill_name, "skill_type",   skill_type))
            {
                ROS_RED_STREAM("No param "<<req.action_name<<"/"<<skill_name<<"/skill_type, skill explore finish");
                res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
                return true;
            }
            skill_type_map.insert(std::make_pair(skill_name,skill_type));
            skill_names.push_back(skill_name);
        }
    }

    for ( const std::string skill_name: skill_names)
    {
        ROS_BOLDMAGENTA_STREAM("Skill name: "<<skill_name.c_str());

        if (!getParam(req.action_name, skill_name, "executed",   executed))
        {
            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/executed");
        }
        else{
            if ( executed )
            {
                if (!getParam(req.action_name, skill_name, "skill_type", skill_type_map.at(skill_name)))
                {
                    ROS_RED_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/skill_type, skill arbitration finish");
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
                    return true;
                }
                ROS_WHITE_STREAM("Skill type: "<<skill_type_map.at(skill_name).c_str());

                if ( skills_evaluation_parameters_.find(skill_type_map.at(skill_name)) == skills_evaluation_parameters_.end() )
                {
                    ROS_RED_STREAM("/"<<skill_type_map.at(skill_name)<<" skill type  is not in the list, skill arbitration finish");
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
                    return true;
                }

                double reward = 0.0;

                for (int i = 0; i < skills_evaluation_parameters_[skill_type_map.at(skill_name)].size(); i++)
                {
                    double value;
                    if ( !getParam(req.action_name,skill_name,skills_evaluation_parameters_[skill_type_map.at(skill_name)].at(i),value) )
                    {
                        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/"<<skills_evaluation_parameters_[skill_type_map.at(skill_name)].at(i) );
                        setParam(req.action_name,skill_name,skills_evaluation_parameters_[skill_type_map.at(skill_name)].at(i),0.0);
                        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/"<<skills_evaluation_parameters_[skill_type_map.at(skill_name)].at(i)<<": "<<0.0);
                        value = 0.0;
                        setParam(req.action_name,skill_name,skills_evaluation_parameters_[skill_type_map.at(skill_name)].at(i),value);
                    }
                    ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<skill_name<<"/"<<skills_evaluation_parameters_[skill_type_map.at(skill_name)].at(i)<<": "<<value);
                    ROS_WHITE_STREAM("Reward + "<<value<<" * "<<skills_evaluation_weights_[skill_type_map.at(skill_name)].at(i));
                    reward = reward + ( value * skills_evaluation_weights_[skill_type_map.at(skill_name)].at(i) );
                }

                setParam(req.action_name,skill_name,"reward",reward);
                ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/reward: "<<reward);
            }
        }
    }
    ROS_BOLDMAGENTA_STREAM("Arbitrator has finished.");

    return true;
}

} // end namespace skills_arbitrator
