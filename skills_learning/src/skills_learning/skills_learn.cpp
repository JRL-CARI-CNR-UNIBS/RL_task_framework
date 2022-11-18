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
    ROS_BLUE_STREAM("Action to explore: "<<req.action_name);
    std::vector<std::string> skill_names;

    if (!getParam(req.action_name, "skill_names", skill_names))
    {
        ROS_WARN("No param: %s/skill_names, skill explore finish", req.action_name.c_str());
        res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
        return true;
    }

    std::map<std::string,std::string> skill_type_map;
    std::string skill_type;
    std::pair <std::string,std::string> skill_pair;

    for (const std::string skill_name: skill_names)
    {
        if (!getParam(req.action_name, skill_name, "skill_type",   skill_type))
        {
            ROS_WARN("No param %s/%s/skill_type, skill explore finish", req.action_name.c_str(), skill_name.c_str());
            res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
            return true;
        }
        skill_pair = std::make_pair(skill_name,skill_type);
        skill_type_map.insert(skill_pair);
    }

    ROS_INFO_STREAM("Skill_type_map size:"<<skill_type_map.size());

    for (const std::string skill_name: skill_names)
    {
        ROS_BLUE_STREAM("Skill requested: /"<<req.action_name<<"/"<<skill_name<<"");
//        if (!getParam(req.action_name, skill_name, "skill_type",   skill_type))
//        {
//            ROS_WARN("No param %s/%s/skill_type, skill explore finish", req.action_name.c_str(), skill_name.c_str());
//            res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
//            return true;
//        }
//        ROS_BLUE_STREAM("Skill type: "<<skill_type);
        ROS_BLUE_STREAM("Skill type: "<<skill_type_map.at(skill_name));

//        if ( skill_execution_parameters_.find(skill_type) == skill_execution_parameters_.end() )
        if ( skill_execution_parameters_.find(skill_type_map.at(skill_name)) == skill_execution_parameters_.end() )
        {
            ROS_WARN("%s skill type  is not in the list, skill explore finish", skill_type.c_str());
            res.result = skills_learning_msgs::SkillLearningResponse::NoSkillType;
            return true;
        }

//        res.result = explore(req.action_name,skill_name,skill_execution_parameters_.at(skill_type));
        res.result = explore(req.action_name,skill_name,skill_execution_parameters_.at(skill_type_map.at(skill_name)));
    }

    ROS_BLUE_STREAM("Skill explore return true");
    return true;
}

bool SkillsLearn::skillsLearning(skills_learning_msgs::SkillLearning::Request  &req,
                                 skills_learning_msgs::SkillLearning::Response &res)
{
    ROS_GREEN_STREAM("Action to learn: "<<req.action_name);
    std::vector<std::string> skill_names;

    if (!getParam(req.action_name, "skill_names", skill_names))
    {
        ROS_WARN("No param: %s/skill_names, skill learning finish", req.action_name.c_str());
        res.result = skills_learning_msgs::SkillLearningResponse::NoParam;
        return true;
    }

    if ( !getParam(req.action_name, "total_reward", total_reward_) )
    {
        ROS_WARN("The parameter %s/total_reward is not set", req.action_name.c_str());
        setParam(req.action_name, "total_reward", 0.0);
        ROS_GREEN_STREAM("Set /"<<req.action_name<<"/total_reward: "<<0.0);
        total_reward_ = 0;
    }
    ROS_GREEN_STREAM("total_reward: "<<total_reward_);

    if ( !getParam(req.action_name, "total_reward_old", total_reward_old_) )
    {
        ROS_WARN("The parameter %s/total_reward_old is not set", req.action_name.c_str());
        setParam(req.action_name, "total_reward_old", -10000000);
        ROS_GREEN_STREAM("Set /"<<req.action_name<<"/total_reward_old: "<<-10000000);
        total_reward_old_ = -10000000;
    }
    ROS_GREEN_STREAM("total_reward_old: "<<total_reward_old_);

    if( total_reward_old_ > total_reward_)
    {
        ROS_GREEN_STREAM("Total_reward < Total_reward_old");
        ROS_GREEN_STREAM(total_reward_<<" < "<<total_reward_old_);
    }
    else
    {
        ROS_GREEN_STREAM("Total_reward > Total_reward_old");
        ROS_GREEN_STREAM(total_reward_<<" > "<<total_reward_old_);
        setParam(req.action_name, "total_reward_old", total_reward_ );
        ROS_GREEN_STREAM("Set /"<<req.action_name<<"/total_reward_old: "<<total_reward_);
    }

    setParam(req.action_name, "executed", 0);
    ROS_GREEN_STREAM("Set /"<<req.action_name<<"/executed: "<<0);

    for (const std::string skill_name: skill_names)
    {
        ROS_GREEN_STREAM("Skill requested: /"<<req.action_name<<"/"<<skill_name);
        std::string skill_type;
        if (!getParam(req.action_name, skill_name, "skill_type", skill_type))
        {
            ROS_WARN("No param: %s/%s, skill learning finish", req.action_name.c_str(), skill_name.c_str());
            res.result = skills_learning_msgs::SkillLearningResponse::NoParam;
            return true;
        }
        ROS_GREEN_STREAM("Skill type: "<<skill_type);

        if ( skill_execution_parameters_.find(skill_type) == skill_execution_parameters_.end() )
        {
            ROS_WARN("%s skill type  is not in the list, skill learning finish", skill_type.c_str());
            res.result = skills_learning_msgs::SkillLearningResponse::NoSkillType;
            return true;
        }

        res.result = learning(req.action_name,skill_name,skill_execution_parameters_.at(skill_type));
    }

    ROS_GREEN_STREAM("Skill learning return true");
    return true;
}

int SkillsLearn::explore (const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    ROS_INFO("Explore 01");
    std::vector<double> param, param_max_var, param_ratio, new_param;
    std::vector<int> param_test_number, new_param_test_number;
    std::string name_max_var, name_ratio, name_test_number;
    int test_number;

    if ( !getParam(action_name, skill_name, "test_number", test_number) )
    {
        ROS_WARN("The parameter %s/%s/test_number is not set, it is considered equal to 0", action_name.c_str(), skill_name.c_str());
        test_number = 0;
    }

    ROS_INFO("Explore 02");
    if ( test_number == 0 )
    {
        test_number++;
        setParam(action_name, skill_name, "test_number", test_number);
        ROS_BLUE_STREAM("/"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
        ROS_BLUE_STREAM("This is the first execution, the params won't be changed");
        return skills_learning_msgs::SkillExploreResponse::Success;
    }

    ROS_INFO("Explore 03");
    for (std::string name: params_name)
    {
        name_max_var = name;
        name_max_var.append("_max_variation");
        name_ratio = name;
        name_ratio.append("_ratio");
        name_test_number = name;
        name_test_number.append("_test_number");

        ROS_INFO("Explore 04");
        if ( !getParam(action_name, skill_name, name, param) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set", action_name.c_str(), skill_name.c_str(), name.c_str());
            return skills_learning_msgs::SkillExploreResponse::NoParam;
        }
//        setParam(action_name, skill_name, name_old, param);
        new_param = param;

        ROS_INFO("Explore 05");
        if ( !getParam(action_name, skill_name, name_max_var, param_max_var) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name_max_var.c_str());
            return skills_learning_msgs::SkillExploreResponse::NoParam;
        }

        ROS_INFO("Explore 06");
        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set, set all ratio to 1", action_name.c_str(), skill_name.c_str(), name_ratio.c_str());
            for (std::size_t i = 0; i < param.size(); i++)
            {
                param_ratio.at(i) = 1.0;
            }
        }

        ROS_INFO("Explore 07");
        ROS_INFO_STREAM("Search: /"<<action_name<<"/"<<skill_name<<"/"<<name_test_number);
        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_INFO_STREAM("Apppapappapa");
            ROS_WARN("The parameter %s/%s/%s is not set, set all test numbers to 0", action_name.c_str(), skill_name.c_str(), name_test_number.c_str());
            param_test_number.clear();
            for (std::size_t i = 0; i < param.size(); i++)
            {
                param_test_number.push_back(0);
                param_test_number.push_back(0);
                param_test_number.push_back(0);
            }
        }
        printArrayParam(name_test_number,param_test_number);

        new_param_test_number = param_test_number;

        ROS_INFO_STREAM("Param test number");
        ROS_INFO_STREAM("Param test number size: "<<param_test_number.size()<<", it should be "<<3*param.size());

        ROS_BLUE_STREAM("/"<<action_name<<"/"<<skill_name<<"/"<<name<<" change:");
        for (std::size_t i = 0; i < param.size(); i++)
        {
            int a = i*3;
            ROS_INFO_STREAM("I: "<<i<<". A: "<<a);
            double tot = param_ratio.at(a) + param_ratio.at(a+1) + param_ratio.at(a+2);
            double random_value = ((rand() % 101) * tot) / 100.0;

            if (random_value < param_ratio.at(a))
            {
                ROS_INFO_STREAM("In -");
                new_param.at(i) = param.at(i) - ((param_max_var.at(i) * (rand() % 100 + 1)) / 100.0);
                new_param_test_number.at(a) = param_test_number.at(a) + 1;
            }
            else if ( random_value > (param_ratio.at(a)+param_ratio.at(a+1)) )
            {
                ROS_INFO_STREAM("In +");
                new_param.at(i) = param.at(i) + ((param_max_var.at(i) * (rand() % 100 + 1)) / 100.0);
                new_param_test_number.at(a+2) = param_test_number.at(a+2) + 1;
            }
            else
            {
                ROS_INFO_STREAM("In =");
                new_param.at(i) = param.at(i);
                new_param_test_number.at(a+1) = param_test_number.at(a+1) + 1;
            }
        }
        ROS_INFO("Explore 08");
        setParam(action_name, skill_name, name, new_param);
        printNewOldParam(name,new_param,param);
        printNewOldParam(name_test_number,new_param_test_number,param_test_number);
        setParam(action_name, skill_name, name_test_number, new_param_test_number);
        ROS_INFO("Explore 09");
    }
    ROS_INFO("Explore 10");
    test_number++;
    setParam(action_name, skill_name, "test_number", test_number);
    ROS_BLUE_STREAM("/"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
    return skills_learning_msgs::SkillExploreResponse::Success;
}

int SkillsLearn::learning(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    ROS_GREEN_STREAM("/"<<action_name<<"/"<<skill_name<<": start learning");
    double reward, reward_old;
    std::vector<double> param, param_ratio, param_old, param_ratio_old;
    std::vector<int> param_test_number;
    int executed, test_number;
    std::string name_old, name_ratio, name_test_number;

    if ( !getParam(action_name, skill_name, "executed", executed))
    {
        ROS_WARN("No param: %s/%s/executed, set to 0", action_name.c_str(), skill_name.c_str());
        executed = 0;
        setParam(action_name, skill_name, "executed", executed);
    }

    if ( !getParam(action_name, skill_name, "test_number", test_number))
    {
        ROS_WARN("No param: %s/%s/test_number, set to 0", action_name.c_str(), skill_name.c_str());
        test_number = 0;
        setParam(action_name, skill_name, "test_number", test_number);
    }

    if (!executed)
    {
        ROS_GREEN_STREAM("/"<<action_name<<"/"<<skill_name<<" skill was not executed, the param return to the old ones");
        if ( test_number < 2)
        {
            ROS_GREEN_STREAM("The skill has not been performed more than once, the parameters cannot be reset");
        }
        else
        {
            for (const std::string name: params_name)
            {
                name_old = name;
                name_old.append("_old");
                name_test_number = name;
                name_test_number.append("_test_number");

                if ( !getParam(action_name, skill_name, name_old, param_old) )
                {
                    ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name_old.c_str());
                    return skills_learning_msgs::SkillLearningResponse::NoParam;
                }
                if ( !getParam(action_name, skill_name, name, param) )
                {
                    ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name.c_str());
                    return skills_learning_msgs::SkillLearningResponse::NoParam;
                }
                ROS_INFO_STREAM("Search: /"<<action_name<<"/"<<skill_name<<"/"<<name_test_number);
                if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
                {
                    ROS_WARN("The parameter /%s/%s/%s is not set, skill learning finish",action_name.c_str(),skill_name.c_str(), name_test_number.c_str());
                    return skills_learning_msgs::SkillLearningResponse::NoParam;
                }
                printArrayParam(name_test_number,param_test_number);

                for (std::size_t i = 0; i < param.size(); i++)
                {
                    int a = i * 3;
                    if ( param.at(i) < param_old.at(i) )
                    {
                        if ( param_test_number.at(a) > 0 )
                        {
                            param_test_number.at(a)--;
                        }
                    }
                    if ( param.at(i) == param_old.at(i) )
                    {
                        if ( param_test_number.at(a+1) > 0 )
                        {
                            param_test_number.at(a+1)--;
                        }
                    }
                    if ( param.at(i) > param_old.at(i) )
                    {
                        if ( param_test_number.at(a+2) > 0 )
                        {
                            param_test_number.at(a+2)--;
                        }
                    }
                }
                setParam(action_name, skill_name, name_test_number, param_test_number);
                ROS_INFO("After test number change");
                printArrayParam(name_test_number,param_test_number);
                setParam(action_name, skill_name, name, param_old);
                ROS_GREEN_STREAM("/"<<action_name<<"/"<<skill_name<<"not executed: old_param->param");
                for (std::size_t i = 0; i < param_old.size(); i++)
                {
                    ROS_GREEN_STREAM(param_old.at(i)<<" -> "<<param.at(i));
                }
                if ( test_number > 0)
                {
                    setParam(action_name, skill_name, "test_number", test_number-1);
                    ROS_GREEN_STREAM("Set /"<<action_name<<"/"<<skill_name<<": "<<test_number<<"->"<<(test_number-1));
                }
            }
        }
        return skills_learning_msgs::SkillLearningResponse::Success;
    }
    else
    {
        if ( test_number == 0 )
        {
            ROS_WARN("Learning not possible, never executed.");
            ROS_WARN("Learning finish");
            return skills_learning_msgs::SkillLearningResponse::Success;
        }
        if ( test_number == 1 )
        {
            ROS_WARN("Learning not possible, only one execution.");
            ROS_WARN("Set old_param with the current one");
            for (const std::string name: params_name)
            {
                name_old = name;
                name_old.append("_old");


                if ( !getParam(action_name, skill_name, name, param) )
                {
                    ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name.c_str());
                    return skills_learning_msgs::SkillLearningResponse::NoParam;
                }
                if ( !getParam(action_name, skill_name, name_old, param_old) )
                {
                    ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name_old.c_str());
                    return skills_learning_msgs::SkillLearningResponse::NoParam;
                }
                printNewOldParam(name,param,param_old);
                printArrayParam(name,param);
                printArrayParam(name_old,param_old);
                setParam(action_name, skill_name, name_old, param);
                printNewOldParam(name,param,param_old);
                printArrayParam(name,param);
                printArrayParam(name_old,param_old);
            }
            setParam(action_name, skill_name, "reward_old", -10000000);
            ROS_YELLOW_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<-10000000);
            ROS_WARN("Learning finish");
            return skills_learning_msgs::SkillLearningResponse::Success;
        }
    }

    setParam(action_name, skill_name, "executed", 0);
    ROS_YELLOW_STREAM("Set /" << action_name << "/" << skill_name << "/executed: " << 0 );

    if ( !getParam(action_name, skill_name, "reward", reward) )
    {
        ROS_WARN("The parameter %s/%s/reward is not set", action_name.c_str(), skill_name.c_str());
        setParam(action_name, skill_name, "reward", 0.0);
        ROS_RED_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<0.0);
        reward = 0;
    }
    ROS_GREEN_STREAM("reward: "<<reward);

    if ( !getParam(action_name, skill_name, "reward_old", reward_old) )
    {
        ROS_WARN("The parameter %s/%s/reward_old is not set", action_name.c_str(), skill_name.c_str());
        setParam(action_name, skill_name, "reward_old", -10000000);
        ROS_RED_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<-10000000);
        reward_old = -10000000;
    }
    ROS_GREEN_STREAM("reward_old: "<<reward_old);

    if ( total_reward_ > total_reward_old_ && reward >= reward_old)
    {
        ROS_GREEN_STREAM("Reward > reward_old, param is approved");
        setParam(action_name, skill_name, "reward_old", reward);
        ROS_RED_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<reward);
    }
    else
    {
        ROS_GREEN_STREAM("Reward < reward_old, param return to old value");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_RED_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }

    for (const std::string name: params_name)
    {
        name_old = name;
        name_old.append("_old");
        name_ratio = name;
        name_ratio.append("_ratio");
        name_test_number = name;
        name_test_number.append("_test_number");

        if ( !getParam(action_name, skill_name, name, param) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name.c_str());
            return skills_learning_msgs::SkillLearningResponse::NoParam;
        }
        if ( !getParam(action_name, skill_name, name_old, param_old) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name_old.c_str());
            return skills_learning_msgs::SkillLearningResponse::NoParam;
        }
        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            ROS_WARN("The parameter %s/%s/%s is not set, skill learning finish", action_name.c_str(), skill_name.c_str(), name_ratio.c_str());
            return skills_learning_msgs::SkillLearningResponse::NoParam;
        }
        param_ratio_old = param_ratio;

        ROS_INFO_STREAM("Search: /"<<action_name<<"/"<<skill_name<<"/"<<name_test_number);
        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_WARN("The parameter /%s/%s/%s is not set, skill learning finish",action_name.c_str(),skill_name.c_str(), name_test_number.c_str());
            return skills_learning_msgs::SkillLearningResponse::NoParam;
        }
        printArrayParam(name_test_number,param_test_number);

        printNewOldParam(name,param,param_old);
        printArrayParam(name,param);
        printArrayParam(name_old,param_old);

        if ( total_reward_ > total_reward_old_ && reward >= reward_old )
        {
            setParam(action_name, skill_name, name_old, param);
            ROS_GREEN_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_old);

            for ( std::size_t i = 0; i < param.size(); i++ )
            {
                int a = i * 3;
                if ( param.at(i) < param_old.at(i) )
                {
                    if (param_test_number.at(a) == 1)
                    {
                        param_ratio.at(a) = 1;
                    }
                    else
                    {
                        param_ratio.at(a) = ((param_ratio.at(a)*(param_test_number.at(a)-1))+1) / param_test_number.at(a);
                    }
                }
                else if ( param.at(i) > param_old.at(i) )
                {
                    if (param_test_number.at(a+2) == 1)
                    {
                        param_ratio.at(a+2) = 1;
                    }
                    else
                    {
                        param_ratio.at(a+2) = ((param_ratio.at(a+2)*(param_test_number.at(a+2)-1))+1) / param_test_number.at(a+2);
                    }
                }
                else
                {
                    if (param_test_number.at(a+1) == 1)
                    {
                        param_ratio.at(a+1) = 1;
                    }
                    else
                    {
                        param_ratio.at(a+1) = ((param_ratio.at(a+1)*(param_test_number.at(a+1)-1))+1) / param_test_number.at(a+1);
                    }
                }
            }
        }
        else
        {
            setParam(action_name, skill_name, name, param_old);
            ROS_GREEN_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name);

            for ( std::size_t i = 0; i < param.size(); i++ )
            {
                int a = i*3;
                if ( param.at(i) < param_old.at(i) )
                {
                    if (param_test_number.at(a) == 1)
                    {
                        param_ratio.at(a) = param_ratio.at(a) / 2;
                    }
                    else
                    {
                        param_ratio.at(a) = (param_ratio.at(a)*(param_test_number.at(a)-1)) / param_test_number.at(a);
                    }
                }
                else if ( param.at(i) > param_old.at(i) )
                {
                    if (param_test_number.at(a+2) == 1)
                    {
                        param_ratio.at(a+2) = param_ratio.at(a+2) / 2;
                    }
                    else
                    {
                        param_ratio.at(a+2) = (param_ratio.at(a+2)*(param_test_number.at(a+2)-1)) / param_test_number.at(a+2);
                    }
                }
                else
                {
                    if (param_test_number.at(a+1) == 1)
                    {
                        param_ratio.at(a+1) = param_ratio.at(a+1) / 2;
                    }
                    else
                    {
                        param_ratio.at(a+1) = (param_ratio.at(a+1)*(param_test_number.at(a+1)-1)) / param_test_number.at(a+1);
                    }
                }
            }
        }
        printNewOldParam(name,param,param_old);
        printArrayParam(name,param);
        printArrayParam(name_old,param_old);

        ROS_INFO(" ");

        printNewOldParam("Param ration",param_ratio,param_ratio_old);
        printArrayParam("Param ration new",param_ratio);
        printArrayParam("Param ration old",param_ratio_old);
        setParam(action_name, skill_name, name_ratio, param_ratio);
        ROS_GREEN_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_ratio);
    }
    return skills_learning_msgs::SkillLearningResponse::Success;
}

void SkillsLearn::printNewOldParam (std::string name, std::vector<double> param, std::vector<double> param_old)
{
    std::string param_str;
    param_str = name;
    param_str.append(" old: [");
    for (std::size_t h = 0; h < param_old.size(); h++)
    {
        if ( h != 0)
        {
            param_str.append(",");
        }
        param_str.append(std::to_string(param_old.at(h)));
    }
    param_str.append("].");
    ROS_INFO("%s", param_str.c_str());
    param_str = name;
    param_str.append(" new: [");
    for (std::size_t h = 0; h < param.size(); h++)
    {
        if ( h != 0)
        {
            param_str.append(",");
        }
        param_str.append(std::to_string(param.at(h)));
    }
    param_str.append("]");
    ROS_INFO("%s", param_str.c_str());
}

void SkillsLearn::printNewOldParam (std::string name, std::vector<int> param, std::vector<int> param_old)
{
    std::string param_str;
    param_str = name;
    param_str.append(" old: [");
    for (std::size_t h = 0; h < param_old.size(); h++)
    {
        if ( h != 0)
        {
            param_str.append(",");
        }
        param_str.append(std::to_string(param_old.at(h)));
    }
    param_str.append("].");
    ROS_INFO("%s", param_str.c_str());
    param_str = name;
    param_str.append(" new: [");
    for (std::size_t h = 0; h < param.size(); h++)
    {
        if ( h != 0)
        {
            param_str.append(",");
        }
        param_str.append(std::to_string(param.at(h)));
    }
    param_str.append("]");
    ROS_INFO("%s", param_str.c_str());
}

void SkillsLearn::printArrayParam (std::string name, std::vector<double> param)
{
    std::string param_str;
    param_str = name;
    param_str.append(": [");
    for (std::size_t h = 0; h < param.size(); h++)
    {
        if ( h != 0)
        {
            param_str.append(",");
        }
        param_str.append(std::to_string(param.at(h)));
    }
    param_str.append("].");
    ROS_INFO("%s", param_str.c_str());
}

void SkillsLearn::printArrayParam (std::string name, std::vector<int> param)
{
    std::string param_str;
    param_str = name;
    param_str.append(": [");
    for (std::size_t h = 0; h < param.size(); h++)
    {
        if ( h != 0)
        {
            param_str.append(",");
        }
        param_str.append(std::to_string(param.at(h)));
    }
    param_str.append("].");
    ROS_INFO("%s", param_str.c_str());
}

} // end namespace skills_executer
