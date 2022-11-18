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
    std::string skill_type, action_type;
    std::vector<std::string> skill_names;

    double total_reward = 0.0;
    int executed;

    if (!getParam(req.action_name, "executed", executed))
    {
        ROS_WARN("No param /%s/executed, skill arbitration finish", req.action_name.c_str());
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    if ( executed )
    {
        if (!getParam(req.action_name, "action_type", action_type))
        {
            ROS_WARN("No param /%s/action_type, skill arbitration finish", req.action_name.c_str());
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
            return true;
        }
        ROS_CYAN_STREAM("Action type: "<<action_type.c_str());

        if ( action_evaluation_parameters_.find(action_type) == action_evaluation_parameters_.end() )
        {
            ROS_WARN("/%s action type is not in the list, skill arbitration finish", action_type.c_str());
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
                ROS_WARN("No param /%s/%s", req.action_name.c_str(), action_evaluation_parameters_[action_type].at(i).c_str() );
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
        ROS_WARN("/%s not executed, skill arbitration finish", req.action_name.c_str());
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::Fail;
        return true;
    }

    if ( !getParam(req.action_name,"skill_names",skill_names) )
    {
        ROS_WARN("No param /%s/skill_names, skill arbitration finish",req.action_name.c_str());
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    for ( const std::string skill_name: skill_names)
    {
        ROS_CYAN_STREAM("Skill name: "<<skill_name.c_str());

        if (!getParam(req.action_name, skill_name, "executed",   executed))
        {
            ROS_WARN("No param /%s/%s/executed", req.action_name.c_str(), skill_name.c_str());
        }
        else{
            if ( executed )
            {
                if (!getParam(req.action_name, skill_name, "skill_type",   skill_type))
                {
                    ROS_WARN("No param /%s/%s/skill_type, skill arbitration finish", req.action_name.c_str(), skill_name.c_str());
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
                    return true;
                }
                ROS_CYAN_STREAM("Skill type: "<<skill_type.c_str());

                if ( skill_evaluation_parameters_.find(skill_type) == skill_evaluation_parameters_.end() )
                {
                    ROS_WARN("/%s skill type  is not in the list, skill arbitration finish", skill_type.c_str());
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
                    return true;
                }

                double reward = 0.0;

//                if (!getParam(req.action_name, skill_name, "reward",   reward))
//                {
//                    ROS_WARN("No param /%s/%s/reward", req.action_name.c_str(), skill_name.c_str());
//                    setParam(req.action_name,skill_name,"reward",0.0);
//                    ROS_RED_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/reward: "<<0.0);
//                    reward = 0.0;
//                }

//                setParam(req.action_name,skill_name,"reward_old",reward);
//                ROS_RED_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/reward_old: "<<reward);

                for (int i = 0; i < skill_evaluation_parameters_[skill_type].size(); i++)
                {
                    double value;
                    if ( !getParam(req.action_name,skill_name,skill_evaluation_parameters_[skill_type].at(i),value) )
                    {
                        ROS_WARN("No param /%s/%s/%s", req.action_name.c_str(), skill_name.c_str(), skill_evaluation_parameters_[skill_type].at(i).c_str() );
                        setParam(req.action_name,skill_name,skill_evaluation_parameters_[skill_type].at(i),0.0);
                        ROS_CYAN_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_parameters_[skill_type].at(i)<<": "<<0.0);
                        value = 0.0;
                    }
                    ROS_CYAN_STREAM("/"<<req.action_name<<"/"<<skill_name<<"/"<<skill_evaluation_parameters_[skill_type].at(i)<<": "<<value);
                    ROS_CYAN_STREAM("Reward + "<<value<<" * "<<skill_evaluation_weight_[skill_type].at(i));
                    reward = reward + ( value * skill_evaluation_weight_[skill_type].at(i) );
                }

                setParam(req.action_name,skill_name,"reward",reward);
                ROS_CYAN_STREAM("Set /"<<req.action_name<<"/"<<skill_name<<"/reward: "<<reward);
            }
        }
    }

    return true;
}

} // end namespace skills_arbitrator
