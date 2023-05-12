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
    ROS_BOLDBLUE_STREAM("Action to explore: "<<req.action_name);
    std::vector<std::string> skill_names;
    std::vector<std::string> params;
    std::map<std::string,std::string> skill_type_map;
    int test_number;

    if ( !getParam(req.action_name,"test_number",test_number) )
    {
        ROS_YELLOW_STREAM("/"<<req.action_name<<"/test_number not set. It considered equal to 0");
        test_number = 0;
        setParam(req.action_name,"test_number",test_number);
    }

    test_number += 1;
    setParam(req.action_name,"test_number",test_number);
    ROS_WHITE_STREAM("/"<<req.action_name<<"/test_number set to "<<test_number);


    if (!n_.getParamNames(params))
    {
        ROS_RED_STREAM("Error with getParamNames");
        res.result = skills_learning_msgs::SkillExploreResponse::Error;
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
                res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
                return true;
            }
            skill_type_map.insert(std::make_pair(skill_name,skill_type));
            skill_names.push_back(skill_name);
        }
    }

    for (const std::string skill_name: skill_names)
    {
        ROS_BOLDBLUE_STREAM("Skill requested: /"<<req.action_name<<"/"<<skill_name<<"");
        ROS_WHITE_STREAM("Skill type: "<<skill_type_map.at(skill_name));

        if ( skill_execution_parameters_.find(skill_type_map.at(skill_name)) == skill_execution_parameters_.end() )
        {
            ROS_RED_STREAM(skill_type_map.at(skill_name)<<" skill type  is not in the list, skill explore finish");
            res.result = skills_learning_msgs::SkillLearningResponse::NoSkillType;
            return true;
        }

        if (!req.exploration_type.compare("standard"))
        {
            res.result = explore(req.action_name,skill_name,skill_execution_parameters_.at(skill_type_map.at(skill_name)));
        }
        else if (!req.exploration_type.compare("variable_range"))
        {
            res.result = explore_variable_range(req.action_name,skill_name,skill_execution_parameters_.at(skill_type_map.at(skill_name)));
        }
        else
        {
            ROS_ERROR("Exploration type error");
            res.result = skills_learning_msgs::SkillLearningResponse::NoParam;
            return true;
        }
    }

    ROS_BLUE_STREAM("Exploration is finished");
    return true;
}

bool SkillsLearn::skillsLearning(skills_learning_msgs::SkillLearning::Request  &req,
                                 skills_learning_msgs::SkillLearning::Response &res)
{
    ROS_GREEN_STREAM("Action to learn: "<<req.action_name);
    std::vector<std::string> skill_names;
    std::vector<std::string> params;
    std::map<std::string,std::string> skill_type_map;
    int test_number, executed;

    if ( !getParam(req.action_name, "executed", executed))
    {
        ROS_YELLOW_STREAM("No param: "<<req.action_name<<"/executed, set to 0");
        executed = 0;
        setParam(req.action_name, "executed", executed);
    }

    if ( executed )
    {
        if ( !getParam(req.action_name, "test_number", test_number))
        {
            ROS_YELLOW_STREAM("No param: "<<req.action_name<<"/test_number, set to 1");
            test_number = 1;
            setParam(req.action_name, "test_number", test_number);
        }
    }
    else
    {
        if ( !getParam(req.action_name, "test_number", test_number))
        {
            ROS_YELLOW_STREAM("No param: "<<req.action_name<<"/test_number, set to 0");
            test_number = 0;
            setParam(req.action_name, "test_number", test_number);
        }
        else
        {
            test_number -= 1;
            ROS_WHITE_STREAM("Action not executedt, /"<<req.action_name<<"/test_number set to "<<test_number);
            setParam(req.action_name, "test_number", test_number);
        }
    }

    if (!n_.getParamNames(params))
    {
        ROS_RED_STREAM("Error with getParamNames");
        res.result = skills_learning_msgs::SkillExploreResponse::Error;
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
                ROS_RED_STREAM("No param "<<req.action_name<<"/"<<skill_name<<"/skill_type, skill learning finish");
                res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
                return true;
            }
            skill_type_map.insert(std::make_pair(skill_name,skill_type));
            skill_names.push_back(skill_name);
        }
    }

    if ( !getParam(req.action_name, "total_reward", total_reward_) )
    {
        ROS_YELLOW_STREAM("The parameter "<<req.action_name<<"/total_reward is not set");
        setParam(req.action_name, "total_reward", -1000000000);
        ROS_YELLOW_STREAM("Set /"<<req.action_name<<"/total_reward: "<<-1000000000);
        total_reward_ = -1000000000;
    }
    ROS_WHITE_STREAM("total_reward: "<<total_reward_);

    if ( !getParam(req.action_name, "total_reward_old", total_reward_old_) )
    {
        ROS_YELLOW_STREAM("The parameter "<<req.action_name<<"/total_reward_old is not set");
        setParam(req.action_name, "total_reward_old", -1000000000);
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/total_reward_old: "<<-1000000000);
        total_reward_old_ = -1000000000;
    }
    ROS_WHITE_STREAM("total_reward_old: "<<total_reward_old_);

    if( total_reward_old_ > total_reward_)
    {
        ROS_WHITE_STREAM("Total_reward < Total_reward_old");
        ROS_WHITE_STREAM(total_reward_<<" < "<<total_reward_old_);
    }
    else
    {
        ROS_WHITE_STREAM("Total_reward > Total_reward_old");
        ROS_WHITE_STREAM(total_reward_<<" > "<<total_reward_old_);
        setParam(req.action_name, "total_reward_old", total_reward_ );
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/total_reward_old: "<<total_reward_);
        setParam(req.action_name, "test_number_star", test_number );
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/test_number_star: "<<test_number);
    }

    setParam(req.action_name, "executed", 0);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/executed: "<<0);

    for (const std::string skill_name: skill_names)
    {
        ROS_GREEN_STREAM("Skill requested: /"<<req.action_name<<"/"<<skill_name);

        if ( skill_execution_parameters_.find(skill_type_map.at(skill_name)) == skill_execution_parameters_.end() )
        {
            ROS_RED_STREAM(skill_type_map.at(skill_name)<<" skill type  is not in the list, skill learning finish");
            res.result = skills_learning_msgs::SkillLearningResponse::NoSkillType;
            return true;
        }

        if (!req.learning_type.compare("standard"))
        {
            res.result = learning(req.action_name,skill_name,skill_execution_parameters_.at(skill_type_map.at(skill_name)));
        }
        else if (!req.learning_type.compare("latest_more_weight"))
        {
            res.result = learning_latest_more_weight(req.action_name,skill_name,skill_execution_parameters_.at(skill_type_map.at(skill_name)));
        }
        else
        {
            ROS_ERROR("Learning type error");
            res.result = skills_learning_msgs::SkillLearningResponse::NoParam;
            return true;
        }
    }

    ROS_GREEN_STREAM("Learning is finished");
    return true;
}

int SkillsLearn::explore(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    std::vector<double> param, param_max_var, param_ratio, new_param;
    std::vector<int> param_test_number, new_param_test_number;
    std::string name_max_var, name_ratio, name_test_number;
    int test_number;

    if ( !getParam(action_name, skill_name, "test_number", test_number) )
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/test_number is not set, it is considered equal to 0");
        test_number = 0;
    }

    if ( test_number == 0 )
    {
        test_number++;
        setParam(action_name, skill_name, "test_number", test_number);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
        ROS_BLUE_STREAM("This is the first execution, the params won't be changed");
        return skills_learning_msgs::SkillExploreResponse::Success;
    }

    for (std::string name: params_name)
    {
        name_max_var = name;
        name_max_var.append("_max_variation");
        name_ratio = name;
        name_ratio.append("_ratio");
        name_test_number = name;
        name_test_number.append("_test_number");

        if ( !getParam(action_name, skill_name, name, param) )
        {
            double d_param;
            if ( !getParam(action_name, skill_name, name, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set, the param won't be changed");
                continue;
            }
            param.clear();
            param.push_back(d_param);
        }
        new_param = param;

        if ( !getParam(action_name, skill_name, name_max_var, param_max_var) )
        {
            double d_param;
            if ( !getParam(action_name, skill_name, name_max_var, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_max_var<<" is not set, the param won't be changed");
                continue;
            }
            param_max_var.clear();
            param_max_var.push_back(d_param);
        }
        if ( param.size() != param_max_var.size() )
        {
            ROS_YELLOW_STREAM("The parameter "<<name<<" and "<<name_max_var<<" have different sizes");
            continue;
        }

        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            param_ratio.clear();
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_ratio<<" is not set, set all ratio to 1");
            for (std::size_t i = 0; i < param.size(); i++)
            {
                param_ratio.push_back(1.0);
                param_ratio.push_back(1.0);
                param_ratio.push_back(1.0);
            }
            setParam(action_name, skill_name, name_ratio, param_ratio);
        }

        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_test_number<<" is not set, set all test numbers to 0");
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

        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"/"<<name<<" change:");
        for (std::size_t i = 0; i < param.size(); i++)
        {
            int a = i*3;
            double tot = param_ratio.at(a) + param_ratio.at(a+1) + param_ratio.at(a+2);
            double random_value = ((rand() % 101) * tot) / 100.0;

            if (random_value < param_ratio.at(a))
            {
                new_param.at(i) = param.at(i) - ((param_max_var.at(i) * (rand() % 100 + 1)) / 100.0);
                new_param_test_number.at(a) = param_test_number.at(a) + 1;
            }
            else if ( random_value > (param_ratio.at(a)+param_ratio.at(a+1)) )
            {
                new_param.at(i) = param.at(i) + ((param_max_var.at(i) * (rand() % 100 + 1)) / 100.0);
                new_param_test_number.at(a+2) = param_test_number.at(a+2) + 1;
            }
            else
            {
                new_param.at(i) = param.at(i);
                new_param_test_number.at(a+1) = param_test_number.at(a+1) + 1;
            }
        }
        if ( new_param.size()==1 )
        {
            setParam(action_name, skill_name, name, new_param.at(0));
        }
        else
        {
            setParam(action_name, skill_name, name, new_param);
        }
        printNewOldParam(name,new_param,param);
        printNewOldParam(name_test_number,new_param_test_number,param_test_number);
        setParam(action_name, skill_name, name_test_number, new_param_test_number);
    }
    test_number++;
    setParam(action_name, skill_name, "test_number", test_number);
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
    return skills_learning_msgs::SkillExploreResponse::Success;
}

int SkillsLearn::learning(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    double reward, reward_old;
    std::vector<double> param, param_ratio, param_old, param_ratio_old;
    std::vector<int> param_test_number;
    int executed, test_number;
    std::string name_old, name_ratio, name_test_number;

    if ( !getParam(action_name, skill_name, "executed", executed))
    {
        ROS_YELLOW_STREAM("No param: "<<action_name<<"/"<<skill_name<<"/executed, set to 0");
        executed = 0;
        setParam(action_name, skill_name, "executed", executed);
    }

    if ( !getParam(action_name, skill_name, "test_number", test_number))
    {
        ROS_YELLOW_STREAM("No param: "<<action_name<<"/"<<skill_name<<"/test_number, set to 0");
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
                    double d_param;
                    if ( !getParam(action_name, skill_name, name_old, d_param) )
                    {
                        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_old<<" is not set");
                        return skills_learning_msgs::SkillLearningResponse::NoParam;
                    }
                    param_old.clear();
                    param_old.push_back(d_param);
                }
                if ( !getParam(action_name, skill_name, name, param) )
                {
                    double d_param;
                    if ( !getParam(action_name, skill_name, name, d_param) )
                    {
                        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set");
                        return skills_learning_msgs::SkillLearningResponse::NoParam;
                    }
                    param.clear();
                    param.push_back(d_param);
                }
                if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
                {
                    ROS_YELLOW_STREAM("The parameter /action_name/skill_name/name_test_number is not set");
                    return skills_learning_msgs::SkillLearningResponse::NoParam;
                }

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
                if ( param_old.size() == 1 )
                {
                    setParam(action_name, skill_name, name, param_old.at(0));
                }
                else
                {
                    setParam(action_name, skill_name, name, param_old);
                }
                ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"not executed: old_param->param");
                printNewOldParam(name,param,param_old);
                if ( test_number > 0)
                {
                    setParam(action_name, skill_name, "test_number", test_number-1);
                    ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<": "<<test_number<<"->"<<(test_number-1));
                }
            }
        }
        return skills_learning_msgs::SkillLearningResponse::Success;
    }
    else
    {
        setParam(action_name, skill_name, "executed", 0);
        ROS_WHITE_STREAM("Set /" << action_name << "/" << skill_name << "/executed: " << 0 );
        if ( test_number == 0 )
        {
            ROS_GREEN_STREAM("Learning not possible, never executed.");
            ROS_GREEN_STREAM("Learning finish");
            return skills_learning_msgs::SkillLearningResponse::Success;
        }
        if ( test_number == 1 )
        {
            ROS_GREEN_STREAM("Learning not possible, only one execution.");
            ROS_GREEN_STREAM("Set old_param with the current one");

            for (const std::string name: params_name)
            {
                name_old = name;
                name_old.append("_old");


                if ( !getParam(action_name, skill_name, name, param) )
                {
                    double d_param;
                    if ( !getParam(action_name, skill_name, name, d_param) )
                    {
                        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set");
                        continue;
                    }
                    param.clear();
                    param.push_back(d_param);
                }
                if ( param.size() == 1 )
                {
                    setParam(action_name, skill_name, name_old, param.at(0));
                }
                else
                {
                    setParam(action_name, skill_name, name_old, param);
                }
                printArrayParam(name,param);
                printArrayParam(name_old,param);
            }
            setParam(action_name, skill_name, "reward_old", -10000000);
            ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<-10000000);
            ROS_GREEN_STREAM("Learning finish");
            return skills_learning_msgs::SkillLearningResponse::Success;
        }
    }

    if ( !getParam(action_name, skill_name, "reward", reward) )
    {
        ROS_YELLOW_STREAM("The parameter /"<<action_name<<"/"<<skill_name<<"/reward is not set");
        reward = 0.0;
        setParam(action_name, skill_name, "reward", reward);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward);
    }
    ROS_WHITE_STREAM("reward: "<<reward);

    if ( !getParam(action_name, skill_name, "reward_old", reward_old) )
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/reward_old is not set");
        reward_old = -10000000;
        setParam(action_name, skill_name, "reward_old", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<reward_old);
    }
    ROS_WHITE_STREAM("reward_old: "<<reward_old);

    if ( total_reward_ > total_reward_old_ && reward >= reward_old)
    {
        ROS_WHITE_STREAM("Total_reward > total_reward_old. Reward >= reward_old. Param is approved");
        ROS_WHITE_STREAM(total_reward_<<" > "<<total_reward_old_<<". "<<reward<<" >= "<<reward_old<<".");
        setParam(action_name, skill_name, "reward_old", reward);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<reward);
    }
    else if ( total_reward_ > total_reward_old_ && reward < reward_old)
    {
        ROS_WHITE_STREAM("Total_reward > total_reward_old. Reward < reward_old. Param not approved");
        ROS_WHITE_STREAM(total_reward_<<" > "<<total_reward_old_<<". "<<reward<<" < "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }
    else if ( total_reward_ <= total_reward_old_ && reward >= reward_old)
    {
        ROS_WHITE_STREAM("Total_reward <= total_reward_old. Reward >= reward_old. Param not approved");
        ROS_WHITE_STREAM(total_reward_<<" <= "<<total_reward_old_<<". "<<reward<<" >= "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }
    else
    {
        ROS_WHITE_STREAM("Total_reward <= total_reward_old. Reward < reward_old. Param not approved");
        ROS_WHITE_STREAM(total_reward_<<" <= "<<total_reward_old_<<". "<<reward<<" < "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
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
            double d_param;
            if ( !getParam(action_name, skill_name, name, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set");
                continue;
            }
            param.clear();
            param.push_back(d_param);
        }
        if ( !getParam(action_name, skill_name, name_old, param_old) )
        {
            double d_param;
            if ( !getParam(action_name, skill_name, name_old, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_old<<" is not set");
                continue;
            }
            param_old.clear();
            param_old.push_back(d_param);
        }
        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_ratio<<" is not set");
            continue;
        }
        param_ratio_old = param_ratio;

        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_YELLOW_STREAM("The parameter /"<<action_name<<"/"<<skill_name<<"/"<<name_test_number<<" is not set");
            continue;
        }
        printArrayParam(name_test_number,param_test_number);

        printNewOldParam(name,param,param_old);
        printArrayParam(name,param);
        printArrayParam(name_old,param_old);

//        if ( total_reward_ > total_reward_old_ && reward >= reward_old )
        if ( total_reward_ > total_reward_old_ )
        {
            if ( param.size() == 1 )
            {
                setParam(action_name, skill_name, name_old, param.at(0));
            }
            else
            {
                setParam(action_name, skill_name, name_old, param);
            }
            ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_old);

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
            if ( param_old.size() == 1 )
            {
                setParam(action_name, skill_name, name, param_old.at(0));
            }
            else
            {
                setParam(action_name, skill_name, name, param_old);
            }
            ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name);

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
        printArrayParam(name,param);
        printArrayParam(name_old,param_old);

        printArrayParam("Param ration new",param_ratio);
        printArrayParam("Param ration old",param_ratio_old);
        setParam(action_name, skill_name, name_ratio, param_ratio);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_ratio);
    }
    return skills_learning_msgs::SkillLearningResponse::Success;
}

int SkillsLearn::explore_variable_range(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    ROS_ERROR('In explore_variable_range');

    std::vector<double> param, param_max_var, param_ratio, new_param;
    std::vector<int> param_test_number, new_param_test_number;
    std::string name_max_var, name_ratio, name_test_number;
    int test_number;

    if ( !getParam(action_name, skill_name, "test_number", test_number) )
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/test_number is not set, it is considered equal to 0");
        test_number = 0;
    }

    if ( test_number == 0 )
    {
        test_number++;
        setParam(action_name, skill_name, "test_number", test_number);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
        ROS_BLUE_STREAM("This is the first execution, the params won't be changed");
        return skills_learning_msgs::SkillExploreResponse::Success;
    }

    for (std::string name: params_name)
    {
        name_max_var = name;
        name_max_var.append("_max_variation");
        name_ratio = name;
        name_ratio.append("_ratio");
        name_test_number = name;
        name_test_number.append("_test_number");

        if ( !getParam(action_name, skill_name, name, param) )
        {
            double d_param;
            if ( !getParam(action_name, skill_name, name, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set, the param won't be changed");
                continue;
            }
            param.clear();
            param.push_back(d_param);
        }
        new_param = param;

        if ( !getParam(action_name, skill_name, name_max_var, param_max_var) )
        {
            double d_param;
            if ( !getParam(action_name, skill_name, name_max_var, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_max_var<<" is not set, the param won't be changed");
                continue;
            }
            param_max_var.clear();
            param_max_var.push_back(d_param);
        }
        if ( param.size() != param_max_var.size() )
        {
            ROS_YELLOW_STREAM("The parameter "<<name<<" and "<<name_max_var<<" have different sizes");
            continue;
        }

        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            param_ratio.clear();
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_ratio<<" is not set, set all ratio to 1");
            for (std::size_t i = 0; i < param.size(); i++)
            {
                param_ratio.push_back(1.0);
                param_ratio.push_back(1.0);
                param_ratio.push_back(1.0);
            }
            setParam(action_name, skill_name, name_ratio, param_ratio);
        }

        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_test_number<<" is not set, set all test numbers to 0");
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

        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"/"<<name<<" change:");
        for (std::size_t i = 0; i < param.size(); i++)
        {
            int a = i*3;
            double tot = param_ratio.at(a) + param_ratio.at(a+1) + param_ratio.at(a+2);
            double random_value = ((rand() % 101) * tot) / 100.0;

            if (random_value < param_ratio.at(a))
            {
                new_param.at(i) = param.at(i) - ((param_max_var.at(i) * param_ratio.at(a) * (rand() % 100 + 1)) / 100.0);
                new_param_test_number.at(a) = param_test_number.at(a) + 1;
            }
            else if ( random_value > (param_ratio.at(a)+param_ratio.at(a+1)) )
            {
                new_param.at(i) = param.at(i) + ((param_max_var.at(i) * param_ratio.at(a) * (rand() % 100 + 1)) / 100.0);
                new_param_test_number.at(a+2) = param_test_number.at(a+2) + 1;
            }
            else
            {
                new_param.at(i) = param.at(i);
                new_param_test_number.at(a+1) = param_test_number.at(a+1) + 1;
            }
        }
        if ( new_param.size()==1 )
        {
            setParam(action_name, skill_name, name, new_param.at(0));
        }
        else
        {
            setParam(action_name, skill_name, name, new_param);
        }
        printNewOldParam(name,new_param,param);
        printNewOldParam(name_test_number,new_param_test_number,param_test_number);
        setParam(action_name, skill_name, name_test_number, new_param_test_number);
    }
    test_number++;
    setParam(action_name, skill_name, "test_number", test_number);
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
    return skills_learning_msgs::SkillExploreResponse::Success;
}

int SkillsLearn::learning_latest_more_weight(const std::string &action_name, const std::string &skill_name, const std::vector<std::string> &params_name)
{
    ROS_ERROR("In learning_latest_more_weight");
    double reward, reward_old;
    std::vector<double> param, param_ratio, param_old, param_ratio_old;
    std::vector<int> param_test_number;
    int executed, test_number;
    std::string name_old, name_ratio, name_test_number;

    if ( !getParam(action_name, skill_name, "executed", executed))
    {
        ROS_YELLOW_STREAM("No param: "<<action_name<<"/"<<skill_name<<"/executed, set to 0");
        executed = 0;
        setParam(action_name, skill_name, "executed", executed);
    }

    if ( !getParam(action_name, skill_name, "test_number", test_number))
    {
        ROS_YELLOW_STREAM("No param: "<<action_name<<"/"<<skill_name<<"/test_number, set to 0");
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
                    double d_param;
                    if ( !getParam(action_name, skill_name, name_old, d_param) )
                    {
                        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_old<<" is not set");
                        return skills_learning_msgs::SkillLearningResponse::NoParam;
                    }
                    param_old.clear();
                    param_old.push_back(d_param);
                }
                if ( !getParam(action_name, skill_name, name, param) )
                {
                    double d_param;
                    if ( !getParam(action_name, skill_name, name, d_param) )
                    {
                        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set");
                        return skills_learning_msgs::SkillLearningResponse::NoParam;
                    }
                    param.clear();
                    param.push_back(d_param);
                }
                if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
                {
                    ROS_YELLOW_STREAM("The parameter /action_name/skill_name/name_test_number is not set");
                    return skills_learning_msgs::SkillLearningResponse::NoParam;
                }

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
                if ( param_old.size() == 1 )
                {
                    setParam(action_name, skill_name, name, param_old.at(0));
                }
                else
                {
                    setParam(action_name, skill_name, name, param_old);
                }
                ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"not executed: old_param->param");
                printNewOldParam(name,param,param_old);
                if ( test_number > 0)
                {
                    setParam(action_name, skill_name, "test_number", test_number-1);
                    ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<": "<<test_number<<"->"<<(test_number-1));
                }
            }
        }
        return skills_learning_msgs::SkillLearningResponse::Success;
    }
    else
    {
        setParam(action_name, skill_name, "executed", 0);
        ROS_WHITE_STREAM("Set /" << action_name << "/" << skill_name << "/executed: " << 0 );
        if ( test_number == 0 )
        {
            ROS_GREEN_STREAM("Learning not possible, never executed.");
            ROS_GREEN_STREAM("Learning finish");
            return skills_learning_msgs::SkillLearningResponse::Success;
        }
        if ( test_number == 1 )
        {
            ROS_GREEN_STREAM("Learning not possible, only one execution.");
            ROS_GREEN_STREAM("Set old_param with the current one");

            for (const std::string name: params_name)
            {
                name_old = name;
                name_old.append("_old");


                if ( !getParam(action_name, skill_name, name, param) )
                {
                    double d_param;
                    if ( !getParam(action_name, skill_name, name, d_param) )
                    {
                        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set");
                        continue;
                    }
                    param.clear();
                    param.push_back(d_param);
                }
                if ( param.size() == 1 )
                {
                    setParam(action_name, skill_name, name_old, param.at(0));
                }
                else
                {
                    setParam(action_name, skill_name, name_old, param);
                }
                printArrayParam(name,param);
                printArrayParam(name_old,param);
            }
            setParam(action_name, skill_name, "reward_old", -10000000);
            ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<-10000000);
            ROS_GREEN_STREAM("Learning finish");
            return skills_learning_msgs::SkillLearningResponse::Success;
        }
    }

    if ( !getParam(action_name, skill_name, "reward", reward) )
    {
        ROS_YELLOW_STREAM("The parameter /"<<action_name<<"/"<<skill_name<<"/reward is not set");
        reward = 0.0;
        setParam(action_name, skill_name, "reward", reward);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward);
    }
    ROS_WHITE_STREAM("reward: "<<reward);

    if ( !getParam(action_name, skill_name, "reward_old", reward_old) )
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/reward_old is not set");
        reward_old = -10000000;
        setParam(action_name, skill_name, "reward_old", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<reward_old);
    }
    ROS_WHITE_STREAM("reward_old: "<<reward_old);

    if ( total_reward_ > total_reward_old_ && reward >= reward_old)
    {
        ROS_WHITE_STREAM("Total_reward > total_reward_old. Reward >= reward_old. Param is approved");
        ROS_WHITE_STREAM(total_reward_<<" > "<<total_reward_old_<<". "<<reward<<" >= "<<reward_old<<".");
        setParam(action_name, skill_name, "reward_old", reward);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<reward);
    }
    else if ( total_reward_ > total_reward_old_ && reward < reward_old)
    {
        ROS_WHITE_STREAM("Total_reward > total_reward_old. Reward < reward_old. Param not approved");
        ROS_WHITE_STREAM(total_reward_<<" > "<<total_reward_old_<<". "<<reward<<" < "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }
    else if ( total_reward_ <= total_reward_old_ && reward >= reward_old)
    {
        ROS_WHITE_STREAM("Total_reward <= total_reward_old. Reward >= reward_old. Param not approved");
        ROS_WHITE_STREAM(total_reward_<<" <= "<<total_reward_old_<<". "<<reward<<" >= "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }
    else
    {
        ROS_WHITE_STREAM("Total_reward <= total_reward_old. Reward < reward_old. Param not approved");
        ROS_WHITE_STREAM(total_reward_<<" <= "<<total_reward_old_<<". "<<reward<<" < "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
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
            double d_param;
            if ( !getParam(action_name, skill_name, name, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name<<" is not set");
                continue;
            }
            param.clear();
            param.push_back(d_param);
        }
        if ( !getParam(action_name, skill_name, name_old, param_old) )
        {
            double d_param;
            if ( !getParam(action_name, skill_name, name_old, d_param) )
            {
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_old<<" is not set");
                continue;
            }
            param_old.clear();
            param_old.push_back(d_param);
        }
        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_ratio<<" is not set");
            continue;
        }
        param_ratio_old = param_ratio;

        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_YELLOW_STREAM("The parameter /"<<action_name<<"/"<<skill_name<<"/"<<name_test_number<<" is not set");
            continue;
        }
        printArrayParam(name_test_number,param_test_number);

        printNewOldParam(name,param,param_old);
        printArrayParam(name,param);
        printArrayParam(name_old,param_old);

//        if ( total_reward_ > total_reward_old_ && reward >= reward_old )
        if ( total_reward_ > total_reward_old_ )
        {
            if ( param.size() == 1 )
            {
                setParam(action_name, skill_name, name_old, param.at(0));
            }
            else
            {
                setParam(action_name, skill_name, name_old, param);
            }
            ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_old);

            for ( std::size_t i = 0; i < param.size(); i++ )
            {
                int a = i * 3;
                if ( param.at(i) < param_old.at(i) )
                {
                    param_ratio.at(a) = ((param_ratio.at(a)*9)+1) / 10;
                }
                else if ( param.at(i) > param_old.at(i) )
                {
                    param_ratio.at(a+2) = ((param_ratio.at(a+2)*9)+1) / 10;
                }
                else
                {
                    param_ratio.at(a+1) = ((param_ratio.at(a+1)*9)+1) / 10;
                }
            }
        }
        else
        {
            if ( param_old.size() == 1 )
            {
                setParam(action_name, skill_name, name, param_old.at(0));
            }
            else
            {
                setParam(action_name, skill_name, name, param_old);
            }
            ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name);

            for ( std::size_t i = 0; i < param.size(); i++ )
            {
                int a = i*3;
                if ( param.at(i) < param_old.at(i) )
                {
                    param_ratio.at(a) = param_ratio.at(a)*0.9;
                }
                else if ( param.at(i) > param_old.at(i) )
                {
                    param_ratio.at(a+2) = param_ratio.at(a+2)*0.9;
                }
                else
                {
                    param_ratio.at(a+1) = param_ratio.at(a+1)*0.9;
                }
            }
        }
        printArrayParam(name,param);
        printArrayParam(name_old,param_old);

        printArrayParam("Param ration new",param_ratio);
        printArrayParam("Param ration old",param_ratio_old);
        setParam(action_name, skill_name, name_ratio, param_ratio);
        ROS_WHITE_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_ratio);
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
