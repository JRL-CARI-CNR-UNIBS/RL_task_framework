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
    ROS_INFO("Action to evaluate: %s", req.action_name.c_str());
    std::string skill_type, action_type;
    std::vector<std::string> skill_names;

    double total_reward = 0.0;
    int executed;

    if (!getParam(req.action_name, "executed", executed))
    {
        ROS_WARN("No param: /%s/executed", req.action_name.c_str());
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    if ( executed )
    {
        if (!getParam(req.action_name, "action_type", action_type))
        {
            ROS_WARN("No param /%s/action_type", req.action_name.c_str());
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
            return true;
        }
        ROS_INFO("Action type: %s", action_type.c_str());

        if ( action_evaluation_parameters_.find(action_type) == action_evaluation_parameters_.end() )
        {
            ROS_WARN("%s action type is not in the list", action_type.c_str());
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
            return true;
        }

        if (!getParam(req.action_name, "total_reward", total_reward))
        {
            ROS_WARN("No param /%s/total_reward", req.action_name.c_str());
            setParam(req.action_name,"total_reward",0.0);
        }

        if (!getParam(req.action_name, "total_reward", total_reward))
        {
            ROS_WARN("No param /%s/total_reward", req.action_name.c_str());
            setParam(req.action_name,"total_reward",0.0);
        }

        ROS_DEBUG("%s/total_reward_old: %lf", req.action_name.c_str(),total_reward);
        setParam(req.action_name,"total_reward_old",total_reward);

        total_reward = 0.0;

        for (int i = 0; i < action_evaluation_parameters_[action_type].size(); i++)
        {
            double value;
            getParam(req.action_name,action_evaluation_parameters_[action_type].at(i),value);
            total_reward = total_reward + ( value * action_evaluation_weight_[action_type].at(i) );
        }

        ROS_DEBUG("%s/reward: %lf", req.action_name.c_str(),total_reward);
        setParam(req.action_name,"total_reward",total_reward);
        setParam(req.action_name,"fail", 0);
    }

    if ( !getParam(req.action_name,"skill_names",skill_names) )
    {
        ROS_WARN("No param /%s/skill_names", req.action_name.c_str());
        res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
        return true;
    }

    for ( const std::string skill_name: skill_names)
    {
        if (!getParam(req.action_name, skill_name, "executed",   executed))
        {
            ROS_WARN("No param: /%s/%s/executed", req.action_name.c_str(), skill_name.c_str());
            res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
            return true;
        }
        else{
            if ( executed )
            {
                if (!getParam(req.action_name, skill_name, "skill_type",   skill_type))
                {
                    ROS_WARN("No param /%s/%s/skill_type", req.action_name.c_str(), skill_name.c_str());
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoParam;
                    return true;
                }
                ROS_INFO("Skill type: %s", skill_type.c_str());

                if ( skill_evaluation_parameters_.find(skill_type) == skill_evaluation_parameters_.end() )
                {
                    ROS_WARN("%s skill type  is not in the list", skill_type.c_str());
                    res.result = skills_arbitrator_msgs::SkillArbitrationResponse::NoSkillType;
                    return true;
                }

                double reward;

                if (!getParam(req.action_name, skill_name, "reward",   reward))
                {
                    ROS_WARN("No param /%s/%s/reward", req.action_name.c_str(), skill_name.c_str());
                    setParam(req.action_name,skill_name,"reward",0.0);
                }

                ROS_DEBUG("%s/%s/reward_old: %lf", req.action_name.c_str(),skill_name.c_str(),reward);
                setParam(req.action_name,skill_name,"reward_old",reward);
                reward = 0.0;

                for (int i = 0; i < skill_evaluation_parameters_[skill_type].size(); i++)
                {
                    double value;
                    getParam(req.action_name,skill_name,skill_evaluation_parameters_[skill_type].at(i),value);
                    reward = reward + ( value * skill_evaluation_weight_[skill_type].at(i) );
                }

                ROS_DEBUG("%s/%s/reward: %lf", req.action_name.c_str(),skill_name.c_str(),reward);
                setParam(req.action_name,skill_name,"reward",reward);
            }
        }
    }

    return true;
}


} // end namespace skills_arbitrator
