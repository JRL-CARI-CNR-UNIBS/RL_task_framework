#include <skills_arbitrator/skills_arbit.h>

namespace skills_arbitrator
{

SkillsArbit::SkillsArbit(const ros::NodeHandle & n) : n_(n)
{
    skill_arbit_srv_ = n_.advertiseService("/skills_arbit/evaluate_skill", &SkillsArbit::skillsArbitration, this);
}

bool SkillsArbit::skillsArbitration(skills_arbitrator_msgs::SkillArbitration::Request  &req,
                                    skills_arbitrator_msgs::SkillArbitration::Response &res)
{
    ROS_CYAN_STREAM("Action to evaluate: "<<req.action_name.c_str());
    std::string action_type;
    std::vector<std::string> skill_names, params;
    std::map<std::string,std::string> skill_type_map;

    double total_reward = 0.0;
    int executed;

    if (!getParam(req.action_name, "executed", executed))
    {
        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/executed, skill arbitration finish");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    if ( executed )
    {
        if (!getParam(req.action_name, "action_type", action_type))
        {
            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/action_type, skill arbitration finish");
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
            return true;
        }
        ROS_CYAN_STREAM("Action type: "<<action_type.c_str());

        if ( action_evaluation_parameters_.find(action_type) == action_evaluation_parameters_.end() )
        {
            ROS_YELLOW_STREAM("/"<<action_type<<" action type is not in the list, skill arbitration finish");
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
            return true;
        }

        total_reward = 0.0;
        ROS_CYAN_STREAM("Start total reward: 0.0");

        for (int i = 0; i < action_evaluation_parameters_[action_type].size(); i++)
        {
            double value;
            if ( !getParam(req.action_name,action_evaluation_parameters_[action_type].at(i),value) )
            {
                ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<action_evaluation_parameters_[action_type].at(i));
                setParam(req.action_name,action_evaluation_parameters_[action_type].at(i),0.0);
                ROS_CYAN_STREAM("Set /"<<req.action_name<<"/"<<action_evaluation_parameters_[action_type].at(i)<<": "<<0.0);
                value = 0.0;
            }
            ROS_CYAN_STREAM("/"<<req.action_name<<"/"<<action_evaluation_parameters_[action_type].at(i)<<": "<<value);
            ROS_CYAN_STREAM("Total_reward + "<<value<<" * "<<action_evaluation_weight_[action_type].at(i));
            total_reward = total_reward + ( value * action_evaluation_weight_[action_type].at(i) );
        }

        setParam(req.action_name,"total_reward",total_reward);
        ROS_CYAN_STREAM("Set /"<<req.action_name<<"/total_reward: "<<total_reward);

        for (int i = 0; i < action_evaluation_parameters_[action_type].size(); i++)
        {
            setParam(req.action_name,action_evaluation_parameters_[action_type].at(i), 0);
            ROS_CYAN_STREAM("Set /"<<req.action_name<<"/"<<action_evaluation_parameters_[action_type].at(i)<<": "<<0);
        }
    }
    else
    {
        ROS_YELLOW_STREAM("/"<<req.action_name<<" not executed, skill arbitration finish");
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Fail;
        return true;
    }

    if (!n_.getParamNames(params))
    {
        ROS_ERROR("Error with getParamNames");
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
                ROS_YELLOW_STREAM("No param "<<req.action_name<<"/"<<skill_name<<"/skill_type, skill explore finish");
                res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
                return true;
            }
            skill_type_map.insert(std::make_pair(skill_name,skill_type));
            skill_names.push_back(skill_name);
        }
    }

    for ( const std::string skill_name: skill_names)
    {
        ROS_CYAN_STREAM("Skill name: "<<skill_name.c_str());

        if (!getParam(req.action_name, skill_name, "executed",   executed))
        {
            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/executed");
        }
        else{
            if ( executed )
            {
                if (!getParam(req.action_name, skill_name, "skill_type", skill_type_map.at(skill_name)))
                {
                    ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/skill_type, skill arbitration finish");
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
                    return true;
                }
                ROS_CYAN_STREAM("Skill type: "<<skill_type_map.at(skill_name).c_str());

                if ( skill_evaluation_parameters_.find(skill_type_map.at(skill_name)) == skill_evaluation_parameters_.end() )
                {
                    ROS_YELLOW_STREAM("/"<<skill_type_map.at(skill_name)<<" skill type  is not in the list, skill arbitration finish");
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
                    return true;
                }

                double reward = 0.0;

                for (int i = 0; i < skill_evaluation_parameters_[skill_type_map.at(skill_name)].size(); i++)
                {
                    double value;
                    if ( !getParam(req.action_name,skill_name,skill_evaluation_parameters_[skill_type_map.at(skill_name)].at(i),value) )
                    {
                        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_parameters_[skill_type_map.at(skill_name)].at(i) );
                        setParam(req.action_name,skill_name,skill_evaluation_parameters_[skill_type_map.at(skill_name)].at(i),0.0);
                        ROS_YELLOW_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_parameters_[skill_type_map.at(skill_name)].at(i)<<": "<<0.0);
                        value = 0.0;
                        setParam(req.action_name,skill_name,skill_evaluation_parameters_[skill_type_map.at(skill_name)].at(i),value);
                    }
                    ROS_CYAN_STREAM("/"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_parameters_[skill_type_map.at(skill_name)].at(i)<<": "<<value);
                    ROS_CYAN_STREAM("Reward + "<<value<<" * "<<skill_evaluation_weight_[skill_type_map.at(skill_name)].at(i));
                    reward = reward + ( value * skill_evaluation_weight_[skill_type_map.at(skill_name)].at(i) );
                }

                setParam(req.action_name,skill_name,"reward",reward);
                ROS_CYAN_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/reward: "<<reward);
            }
        }
    }

    return true;
}

} // end namespace skills_arbitrator
