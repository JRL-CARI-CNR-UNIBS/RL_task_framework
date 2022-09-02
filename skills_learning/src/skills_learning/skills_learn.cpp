#include <skills_learning/skills_learn.h>

namespace skills_learning
{

SkillsLearn::SkillsLearn(const ros::NodeHandle & n) : n_(n)
{
    skill_learn_srv_ = n_.advertiseService("/skills_learn/learn_skill", &SkillsLearn::skillsLearning, this);
    skill_explore_srv_ = n_.advertiseService("/skills_learn/explore_skill", &SkillsLearn::skillsExplore, this);
}

bool SkillsLearn::skillsExplore(skills_learning_msgs::SkillExplore::Request  &req,
                                 skills_learning_msgs::SkillExplore::Response &res)
{
    ROS_INFO("Action to explore: %s", req.action_name.c_str());
    std::vector<std::string> skill_names;

    if (!getParam(req.action_name, "skill_names", skill_names))
    {
        ROS_WARN("No param: %s/skill_names", req.action_name.c_str());
        res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
        return true;
    }

    for (const std::string skill_name: skill_names)
    {
        ROS_INFO("Skill requested: %s/%s", req.action_name.c_str(), skill_name.c_str());
        std::string skill_type;
        if (!getParam(req.action_name, skill_name, "skill_type",   skill_type))
        {
            ROS_WARN("No param %s/%s/skill_type", req.action_name.c_str(), skill_name.c_str());
            res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
            return true;
        }
        ROS_INFO("Skill type: %s", skill_type.c_str());

        if ( skill_execution_parameters_.find(skill_type) == skill_execution_parameters_.end() )
        {
            ROS_WARN("%s skill type  is not in the list", skill_type.c_str());
            res.result = skills_learning_msgs::SkillLearningResponse::NoSkillType;
            return true;
        }

        res.result = explore(req.action_name,skill_name,skill_execution_parameters_.at(skill_type));
    }

    ROS_INFO("Return true");
    return true;
}

bool SkillsLearn::skillsLearning(skills_learning_msgs::SkillLearning::Request  &req,
                                 skills_learning_msgs::SkillLearning::Response &res)
{
    ROS_INFO("Action to learn: %s", req.action_name.c_str());
    std::vector<std::string> skill_names;

    if (!getParam(req.action_name, "skill_names", skill_names))
    {
        ROS_WARN("No param: %s/skill_names", req.action_name.c_str());
        res.result = skills_learning_msgs::SkillLearningResponse::NoParam;
        return true;
    }

    setParam(req.action_name, "executed", 0);

    for (const std::string skill_name: skill_names)
    {
        ROS_INFO("Skill requested: %s/%s", req.action_name.c_str(), skill_name.c_str());
        std::string skill_type;
        if (!getParam(req.action_name, skill_name, "skill_type", skill_type))
        {
            ROS_WARN("No param: %s/%s", req.action_name.c_str(), skill_name.c_str());
            res.result = skills_learning_msgs::SkillLearningResponse::NoParam;
            return true;
        }
        ROS_INFO("Skill type: %s", skill_type.c_str());

        if ( skill_execution_parameters_.find(skill_type) == skill_execution_parameters_.end() )
        {
            ROS_WARN("%s skill type  is not in the list", skill_type.c_str());
            res.result = skills_learning_msgs::SkillLearningResponse::NoSkillType;
            return true;
        }

        res.result = learning(req.action_name,skill_name,skill_execution_parameters_.at(skill_type));
    }

    ROS_INFO("Return true");
    return true;
}

int SkillsLearn::learning(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    ROS_DEBUG("%s/%s: start learning",action_name.c_str(),skill_name.c_str());
    double reward, reward_old, total_reward, total_reward_old;
    int test_number;

    if ( !getParam(action_name, "total_reward", total_reward) )
    {
        ROS_WARN("The parameter %s/total_reward is not set", action_name.c_str());
        setParam(action_name, "total_reward", 0.0);
        total_reward = 0;
    }

    if ( !getParam(action_name, "total_reward_old", total_reward_old) )
    {
        ROS_WARN("The parameter %s/total_reward_old is not set", action_name.c_str());
        setParam(action_name, "total_reward_old", 0.0);
        total_reward_old = 0;
    }

    if ( !getParam(action_name, skill_name, "reward", reward) )
    {
        ROS_WARN("The parameter %s/%s/reward is not set", action_name.c_str(), skill_name.c_str());
        setParam(action_name, skill_name, "reward", 0.0);
        reward = 0;
    }

    if ( !getParam(action_name, skill_name, "reward_old", reward_old) )
    {
        ROS_WARN("The parameter %s/%s/reward_old is not set", action_name.c_str(), skill_name.c_str());
        setParam(action_name, skill_name, "reward", 0.0);
        reward = 0;
    }

    if ( !getParam(action_name, skill_name, "test_number", test_number))
    {
        ROS_WARN("The parameter %s/%s/test_number is not set", action_name.c_str(), skill_name.c_str());
        setParam(action_name, skill_name, "test_number", 1);
        test_number = 1;
    }

    if ( test_number == 1 )
    {
        return skills_learning_msgs::SkillLearningResponse::Success;
    }

    int executed;

    if ( !getParam(action_name, skill_name, "executed", executed))
    {
        ROS_WARN("No param: %s/%s/executed", action_name.c_str(), skill_name.c_str());
        executed = 0;
    }

    std::vector<double> param, param_old, param_ratio;
    std::string name_old, name_ratio;

    if (!executed)
    {
        ROS_INFO("%s/%s skill was not executed, the param return to the old ones");
        for (const std::string name: params_name)
        {
            name_old = name;
            name_old.append("_old");
            if ( !getParam(action_name, skill_name, name_old, param_old) )
            {
                ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name_old.c_str());
                return skills_learning_msgs::SkillLearningResponse::NoParam;
            }
            setParam(action_name, skill_name, name, param_old);
        }
        return skills_learning_msgs::SkillLearningResponse::Success;
    }

    setParam(action_name, skill_name, "executed", 0);

    if ( total_reward_old > total_reward)
    {
        reward_old = total_reward_old;
        reward     = total_reward;
    }

    for (const std::string name: params_name)
    {
        name_old = name;
        name_old.append("_old");
        name_ratio = name;
        name_ratio.append("_ratio");

        if ( !getParam(action_name, skill_name, name, param) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name.c_str());
            return skills_learning_msgs::SkillLearningResponse::NoParam;
        }
        if ( !getParam(action_name, skill_name, name_old, param_old) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name_old.c_str());
            return skills_learning_msgs::SkillLearningResponse::NoParam;
        }
        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name_ratio.c_str());
            return skills_learning_msgs::SkillLearningResponse::NoParam;
        }

        if (reward_old < reward)
        {
            ROS_DEBUG("Reward_old < reward");
            for ( std::size_t i = 0; i < param.size(); i++ )
            {
                int a = i*3;
                if ( param.at(i) < param_old.at(i) )
                {
                    ROS_DEBUG("Param : %s", name.c_str());
                    ROS_DEBUG("Param < param_old: %lf < %lf", param.at(i), param_old.at(i));
                    ROS_DEBUG("Initial param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                    if (test_number == 1)
                    {
                        param_ratio.at(a) = param_ratio.at(a) / 2;
                    }
                    else
                    {
                        param_ratio.at(a) = (param_ratio.at(a)*(test_number-1)) / test_number;
                    }
                    ROS_DEBUG("Final param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                }
                else if ( param.at(i) > param_old.at(i) )
                {
                    ROS_DEBUG("Param : %s", name.c_str());
                    ROS_DEBUG("Param > param_old: %lf > %lf", param.at(i), param_old.at(i));
                    ROS_DEBUG("Initial param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                    if (test_number == 1)
                    {
                        param_ratio.at(a+2) = param_ratio.at(a+2) / 2;
                    }
                    else
                    {
                        param_ratio.at(a+2) = (param_ratio.at(a+2)*(test_number-1)) / test_number;
                    }
                    ROS_DEBUG("Final param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                }
                else
                {
                    ROS_DEBUG("Param : %s", name.c_str());
                    ROS_DEBUG("Param = param_old: %lf = %lf", param.at(i), param_old.at(i));
                    ROS_DEBUG("Initial param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                    if (test_number == 1)
                    {
                        param_ratio.at(a+1) = param_ratio.at(a+1) / 2;
                    }
                    else
                    {
                        param_ratio.at(a+1) = (param_ratio.at(a+1)*(test_number-1)) / test_number;
                    }
                    ROS_DEBUG("Final param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                }
            }
            setParam(action_name, skill_name, name_ratio, param_ratio);
            setParam(action_name, skill_name, name, param_old);
        }
        else
        {
            ROS_DEBUG("Reward_old >= reward");
            for ( std::size_t i = 0; i < param.size(); i++ )
            {
                int a = i * 3;
                if ( param.at(i) < param_old.at(i) )
                {
                    ROS_DEBUG("Param : %s", name.c_str());
                    ROS_DEBUG("Param < param_old: %lf < %lf", param.at(i), param_old.at(i));
                    ROS_DEBUG("Initial param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                    if (test_number == 1)
                    {
                        param_ratio.at(a) = 1;
                    }
                    else
                    {
                        param_ratio.at(a) = ((param_ratio.at(a)*(test_number-1))+1) / test_number;
                    }
                    ROS_DEBUG("Final param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                }
                else if ( param.at(i) > param_old.at(i) )
                {
                    ROS_DEBUG("Param : %s", name.c_str());
                    ROS_DEBUG("Param > param_old: %lf > %lf", param.at(i), param_old.at(i));
                    ROS_DEBUG("Initial param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                    if (test_number == 1)
                    {
                        param_ratio.at(a+2) = 1;
                    }
                    else
                    {
                        param_ratio.at(a+2) = ((param_ratio.at(a+2)*(test_number-1))+1) / test_number;
                    }
                    ROS_DEBUG("Final param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                }
                else
                {
                    ROS_DEBUG("Param : %s", name.c_str());
                    ROS_DEBUG("Param = param_old: %lf = %lf", param.at(i), param_old.at(i));
                    ROS_DEBUG("Initial param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                    if (test_number == 1)
                    {
                        param_ratio.at(a+1) = 1;
                    }
                    else
                    {
                        param_ratio.at(a+1) = ((param_ratio.at(a+1)*(test_number-1))+1) / test_number;
                    }
                    ROS_DEBUG("Final param_ratio: [%lf,%lf,%lf]", param_ratio.at(a), param_ratio.at(a+1), param_ratio.at(a+2));
                }
            }
            setParam(action_name, skill_name, name_ratio, param_ratio);
            setParam(action_name, skill_name, "reward", reward_old);
        }
    }
    return skills_learning_msgs::SkillLearningResponse::Success;
}

int SkillsLearn::explore (const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    std::vector<double> param, param_max_var, param_ratio, new_param;
    std::string name_max_var, name_ratio, name_old;

    int test_number;

    if ( !getParam(action_name, skill_name, "test_number", test_number) )
    {
        ROS_WARN("The parameter %s/%s/test_number is not set", action_name.c_str(), skill_name.c_str());
        return skills_learning_msgs::SkillExploreResponse::NoParam;
    }

    if ( test_number == 0 )
    {
        test_number++;
        setParam(action_name, skill_name, "test_number", test_number);
        return skills_learning_msgs::SkillExploreResponse::Success;
    }

    for (std::string name: params_name)
    {
        name_max_var = name;
        name_max_var.append("_max_variation");
        name_ratio = name;
        name_ratio.append("_ratio");
        name_old = name;
        name_old.append("_old");

        if ( !getParam(action_name, skill_name, name, param) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name.c_str());
            return skills_learning_msgs::SkillExploreResponse::NoParam;
        }
        setParam(action_name, skill_name, name_old, param);
        new_param = param;

        if ( !getParam(action_name, skill_name, name_max_var, param_max_var) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name_max_var.c_str());
            return skills_learning_msgs::SkillExploreResponse::NoParam;
        }

        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name_ratio.c_str());
            return skills_learning_msgs::SkillExploreResponse::NoParam;
        }

        for (std::size_t i = 0; i < param.size(); i++)
        {
            int a = i*3;
            int tot = param_ratio.at(a) + param_ratio.at(a+1) + param_ratio.at(a+2);
            int random_value = ((rand() % 101) * tot) / 100;

            if (random_value < param_ratio.at(a))
            {
                new_param.at(i) = param.at(i) - ((param_max_var.at(i) * (rand() % 100 + 1)) / 100);
            }
            else if ( random_value > (param_ratio.at(a)+param_ratio.at(a+1)) )
            {
                new_param.at(i) = param.at(i) + ((param_max_var.at(i) * (rand() % 100 + 1)) / 100);
            }
        }
        setParam(action_name, skill_name, name, new_param);
    }
    test_number++;
    setParam(action_name, skill_name, "test_number", test_number);
    return skills_learning_msgs::SkillExploreResponse::Success;
}

} // end namespace skills_executer
