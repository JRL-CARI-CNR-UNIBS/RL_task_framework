#include <skills_learning/skills_learn.h>

namespace skills_learning
{

SkillsLearn::SkillsLearn(const ros::NodeHandle & n) : n_(n)
{
    if (!n_.getParam("/skills_executer/skills_parameters_name_space", exec_param_ns_))
    {
        ROS_WARN_STREAM("No /skills_executer/skills_parameters_name_space param, defaul 'exec_params'");
        exec_param_ns_ = "exec_params";
    }

    if (!n_.getParam("/skills_learning/learning_parameter_name_space", learn_param_ns_))
    {
        ROS_WARN_STREAM("No /skills_learning/learning_parameter_name_space param, defaul 'learn_params'");
        learn_param_ns_ = "learn_params";
    }

    //
    XmlRpc::XmlRpcValue actions_param;
    if (!n_.getParam("/" + learn_param_ns_ + "/actions", actions_param))
    {
        ROS_ERROR_STREAM("No /" + learn_param_ns_ + "/actions param");
        return;
    }
    std::vector<std::string> action_names = skills_util::getMemberByXml(actions_param);
    for (const std::string action_name: action_names)
    {
        XmlRpc::XmlRpcValue skills_param;
        if (!n_.getParam("/" + learn_param_ns_ + "/actions/" + action_name + "/skills", skills_param))
        {
            ROS_ERROR_STREAM("No /" + learn_param_ns_ + "/actions/" + action_name + "/skills param");
            return;
        }
        std::vector<std::string> skill_names = skills_util::getMemberByXml(skills_param);
        std::map<std::string,std::map<std::string,std::vector<std::vector<double>>>> skill_name_to_params;
        std::map<std::string,std::map<std::string,std::vector<double>>> skill_name_to_max_var;
        for (const std::string skill_name: skill_names)
        {
            XmlRpc::XmlRpcValue params;
            if (!n_.getParam("/" + learn_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name, params))
            {
                ROS_ERROR_STREAM("No /" + learn_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + " param");
                return;
            }
            std::vector<std::string> param_names = skills_util::getMemberByXml(params);
            std::map<std::string,std::vector<std::vector<double>>> param_name_to_param_value;
            std::map<std::string,std::vector<double>> param_name_to_param_max_variation;
            for (const std::string param_name: param_names)
            {
                std::vector<std::vector<double>> param_value;
                std::vector<double> param_max_variation;
                XmlRpc::XmlRpcValue param_xml;
                if (!n_.getParam("/" + learn_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + "/" + param_name, param_xml))
                {
                    ROS_ERROR_STREAM("No /" + learn_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + "/" + param_name + " param");
                    return;
                }
                else
                {
                    if (param_xml.getType() == XmlRpc::XmlRpcValue::Type::TypeArray)
                    {
                        if ((param_xml[0].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble || param_xml[0].getType() == XmlRpc::XmlRpcValue::Type::TypeInt) && param_xml.size() == 2)
                        {
                            std::vector<double> bound;
                            if (param_xml[0].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
                            {
                                bound.push_back(double(param_xml[0]));
                            }
                            else
                            {
                                bound.push_back(double(int(param_xml[0])));
                            }
                            if (param_xml[1].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
                            {
                                bound.push_back(double(param_xml[1]));
                            }
                            else
                            {
                                bound.push_back(double(int(param_xml[1])));
                            }
                            param_value.push_back(bound);
                            param_max_variation.push_back((bound.at(1)-bound.at(0))/parameter_space_division_);
                        }
                        else if (param_xml[0].getType() == XmlRpc::XmlRpcValue::Type::TypeArray)
                        {
                            for (std::size_t i = 0; i < param_xml.size(); i++)
                            {
                                if ((param_xml[i][0].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble || param_xml[i][0].getType() == XmlRpc::XmlRpcValue::Type::TypeInt) && param_xml[i].size() == 2)
                                {
                                    std::vector<double> bound;
                                    if (param_xml[i][0].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
                                    {
                                        bound.push_back(double(param_xml[i][0]));
                                    }
                                    else
                                    {
                                        bound.push_back(double(int(param_xml[i][0])));
                                    }
                                    if (param_xml[i][1].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
                                    {
                                        bound.push_back(double(param_xml[i][1]));
                                    }
                                    else
                                    {
                                        bound.push_back(double(int(param_xml[i][1])));
                                    }
                                    param_value.push_back(bound);
                                    param_max_variation.push_back((bound.at(1)-bound.at(0))/parameter_space_division_);
                                }
                                else
                                {
                                    ROS_ERROR_STREAM("Something wrong with the parameters format.");
                                    return;
                                }
                            }
                        }
                        else
                        {
                            ROS_ERROR_STREAM("Something wrong with the parameters format.");
                            return;
                        }
                    }
                    else
                    {
                        ROS_ERROR_STREAM("The /" + learn_param_ns_ + "/actions/" + action_name + "/skills/" + skill_name + "/" + param_name + " param isn't an array.");
                        return;
                    }
                }
                param_name_to_param_value.insert(std::make_pair(param_name,param_value));
                param_name_to_param_max_variation.insert(std::make_pair(param_name,param_max_variation));
            }
            skill_name_to_params.insert(std::make_pair(skill_name,param_name_to_param_value));
            skill_name_to_max_var.insert(std::make_pair(skill_name,param_name_to_param_max_variation));
        }
        exec_params_bound_.insert(std::make_pair(action_name,skill_name_to_params));
        exec_params_max_variation_.insert(std::make_pair(action_name,skill_name_to_max_var));
    }

    for (const auto action: exec_params_bound_)
    {
         ROS_INFO_STREAM(action.first + ":");
         for (const auto skill: action.second)
         {
             ROS_INFO_STREAM("  " +skill.first+":");
             for (const auto param: skill.second)
             {
                 ROS_INFO_STREAM("    " + param.first + ":");
                 for (const auto vec: param.second)
                 {
                     ROS_INFO_STREAM("     - [" + std::to_string(vec.at(0)) + ", " + std::to_string(vec.at(1)) +"]");
                 }
             }
         }
    }
    for (const auto action: exec_params_max_variation_)
    {
         ROS_INFO_STREAM(action.first + ":");
         for (const auto skill: action.second)
         {
             ROS_INFO_STREAM("  " +skill.first+":");
             for (const auto param: skill.second)
             {
                 ROS_INFO_STREAM("    " + param.first + ":");
                 for (const auto val: param.second)
                 {
                     ROS_INFO_STREAM("     - " + std::to_string(val));
                 }
             }
         }
    }

    //
    skill_learn_srv_ = n_.advertiseService("/skills_learn/learn_skill", &SkillsLearn::skillsLearning, this);
    skill_explore_srv_ = n_.advertiseService("/skills_learn/explore_skill", &SkillsLearn::skillsExplore, this);
}

bool SkillsLearn::skillsExplore(skills_learning_msgs::SkillExplore::Request  &req,
                                 skills_learning_msgs::SkillExplore::Response &res)
{
    ROS_BOLDBLUE_STREAM("Action to explore: "<<req.action_name);
    int test_number;

    if ( !getParam(req.action_name,"test_number",test_number) )
    {
        ROS_WARN_STREAM("/"<<req.action_name<<"/test_number not set. It considered equal to 0");
        test_number = 0;
        setParam(req.action_name,"test_number",test_number);
    }

    if (test_number == 0)
    {
        ROS_INFO_STREAM("/"<<req.action_name<<"/test_number equal to 0.");
    }

    test_number += 1;
    setParam(req.action_name,"test_number",test_number);
    ROS_INFO_STREAM("/"<<req.action_name<<"/test_number set to "<<test_number);

    if (test_number ==1)
    {
        ROS_INFO_STREAM("This is the first test, the params won't be changed");
        res.result = skills_learning_msgs::SkillLearningResponse::Success;
        return true;
    }

    if (exec_params_bound_.find(req.action_name) == exec_params_bound_.end())
    {
        ROS_INFO_STREAM(req.action_name<<" action doesn't need to modified");
        return true;
    }

    XmlRpc::XmlRpcValue skills_param;
    if (!n_.getParam("/" + exec_param_ns_ + "/actions/" + req.action_name + "/skills", skills_param))
    {
        ROS_ERROR_STREAM("No /" + exec_param_ns_ + "/actions/" + req.action_name + "/skills param");
        res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
        return true;
    }

    std::vector<std::string> skill_names = skills_util::getMemberByXml(skills_param);

    for (const std::string skill_name: skill_names)
    {
        if (exec_params_bound_[req.action_name].find(skill_name) == exec_params_bound_[req.action_name].end())
        {
            ROS_INFO_STREAM(req.action_name + "/" +skill_name<<" skill doesn't need to modified.");
            continue;
        }
        res.result = explore(req.action_name,skill_name,req.exploration_type);
    }
    //
    ROS_BLUE_STREAM("Exploration is finished");
    res.result = skills_learning_msgs::SkillLearningResponse::Success;
    return true;
}

bool SkillsLearn::skillsLearning(skills_learning_msgs::SkillLearning::Request  &req,
                                 skills_learning_msgs::SkillLearning::Response &res)
{
    ROS_GREEN_STREAM("Action to learn: "<<req.action_name);
    int test_number, executed;

    if ( !getParam(req.action_name, "executed", executed))
    {
        ROS_WARN_STREAM("No param: "<<req.action_name<<"/executed, set to 0");
        executed = 0;
        setParam(req.action_name, "executed", executed);
    }

    if ( executed )
    {
        if ( !getParam(req.action_name, "test_number", test_number))
        {
            ROS_WARN_STREAM("No param: "<<req.action_name<<"/test_number, set to 1");
            test_number = 1;
            setParam(req.action_name, "test_number", test_number);
        }
    }
    else
    {
        if ( !getParam(req.action_name, "test_number", test_number))
        {
            ROS_WARN_STREAM("No param: "<<req.action_name<<"/test_number, set to 0");
            test_number = 0;
            setParam(req.action_name, "test_number", test_number);
        }
        else
        {
            test_number -= 1;
            ROS_INFO_STREAM("Action not executedt, /"<<req.action_name<<"/test_number set to "<<test_number);
            setParam(req.action_name, "test_number", test_number);
        }
    }

    if (exec_params_bound_.find(req.action_name) == exec_params_bound_.end())
    {
        ROS_INFO_STREAM(req.action_name<<" action doesn't need to learned");
        return true;
    }

    XmlRpc::XmlRpcValue skills_param;
    if (!n_.getParam("/" + exec_param_ns_ + "/actions/" + req.action_name + "/skills", skills_param))
    {
        ROS_ERROR_STREAM("No /" + exec_param_ns_ + "/actions/" + req.action_name + "/skills param");
        res.result = skills_learning_msgs::SkillExploreResponse::NoParam;
        return true;
    }

    std::vector<std::string> skill_names = skills_util::getMemberByXml(skills_param);

    if ( !getParam(req.action_name, "total_reward", total_reward_) )
    {
        ROS_WARN_STREAM("The parameter "<<req.action_name<<"/total_reward is not set");
        setParam(req.action_name, "total_reward", initial_total_reward_);
        ROS_WARN_STREAM("Set /"<<req.action_name<<"/total_reward: "<<initial_total_reward_);
        total_reward_ = initial_total_reward_;
    }
    ROS_INFO_STREAM("total_reward: "<<total_reward_);

    if ( !getParam(req.action_name, "total_reward_old", total_reward_old_) )
    {
        ROS_WARN_STREAM("The parameter "<<req.action_name<<"/total_reward_old is not set");
        setParam(req.action_name, "total_reward_old", initial_total_reward_);
        ROS_INFO_STREAM("Set /"<<req.action_name<<"/total_reward_old: "<<initial_total_reward_);
        total_reward_old_ = initial_total_reward_;
    }
    ROS_INFO_STREAM("total_reward_old: "<<total_reward_old_);

    if( total_reward_old_ > total_reward_)
    {
        ROS_INFO_STREAM("Total_reward < Total_reward_old");
        ROS_INFO_STREAM(total_reward_<<" < "<<total_reward_old_);
    }
    else
    {
        ROS_INFO_STREAM("Total_reward > Total_reward_old");
        ROS_INFO_STREAM(total_reward_<<" > "<<total_reward_old_);
        setParam(req.action_name, "total_reward_old", total_reward_ );
        ROS_INFO_STREAM("Set /"<<req.action_name<<"/total_reward_old: "<<total_reward_);
        setParam(req.action_name, "test_number_star", test_number );
        ROS_INFO_STREAM("Set /"<<req.action_name<<"/test_number_star: "<<test_number);
    }

    setParam(req.action_name, "executed", 0);
    ROS_INFO_STREAM("Set /"<<req.action_name<<"/executed: "<<0);

    for (const auto skill_name: skill_names)
    {
        ROS_GREEN_STREAM("Skill requested: /"<<req.action_name<<"/"<<skill_name);

        if (exec_params_bound_[req.action_name].find(skill_name) == exec_params_bound_[req.action_name].end())
        {
            ROS_INFO_STREAM(req.action_name + "/" +skill_name<<" skill doesn't need to learned.");
            continue;
        }
        res.result = learning(req.action_name,skill_name,req.learning_type);
    }
    ROS_GREEN_STREAM("Learning is finished");
    return true;
}


//
int SkillsLearn::explore(const std::string &action_name, const std::string &skill_name, const std::string &exploration_type)
{
    std::vector<double> param_value, param_ratio, new_param;
    std::vector<int> param_test_number, new_param_test_number;
    std::string name_ratio, name_test_number;
    int test_number;

    if ( !getParam(action_name, skill_name, "test_number", test_number) )
    {
        ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/test_number is not set, it is considered equal to 0");
        test_number = 0;
    }

    if ( test_number == 0 )
    {
        test_number++;
        setParam(action_name, skill_name, "test_number", test_number);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
        ROS_BLUE_STREAM("This is the first execution, the params won't be changed");
        return skills_learning_msgs::SkillExploreResponse::Success;
    }

    for (const auto param_bound: exec_params_bound_[action_name][skill_name])
    {
        std::string param_name = param_bound.first;
        name_ratio = param_name;
        name_ratio.append("_ratio");
        name_test_number = param_name;
        name_test_number.append("_test_number");

        if ( !getParam(action_name, skill_name, param_name, param_value) )
        {
            double d_param_value;
            if ( !getParam(action_name, skill_name, param_name, d_param_value) )
            {
                ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<param_name<<" is not set, the param won't be changed");
                continue;
            }
            param_value.clear();
            param_value.push_back(d_param_value);
        }
        new_param = param_value;

        if (param_value.size() != param_bound.second.size())
        {
            ROS_WARN_STREAM("The parameter values and its bound have different sizes. Parameter: "<<action_name<<"/"<<skill_name<<"/"<<param_name);
            continue;
        }

        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            param_ratio.clear();
            ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_ratio<<" is not set, set all ratio to 1");
            for (std::size_t i = 0; i < param_value.size(); i++)
            {
                param_ratio.push_back(1.0);
                param_ratio.push_back(1.0);
                param_ratio.push_back(1.0);
            }
            setParam(action_name, skill_name, name_ratio, param_ratio);
        }

        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_test_number<<" is not set, set all test numbers to 0");
            param_test_number.clear();
            for (std::size_t i = 0; i < param_value.size(); i++)
            {
                param_test_number.push_back(0);
                param_test_number.push_back(0);
                param_test_number.push_back(0);
            }
            setParam(action_name, skill_name, name_test_number, param_test_number);
        }
        printArrayParam(name_test_number,param_test_number);
        new_param_test_number = param_test_number;

        ROS_INFO_STREAM("/"<<action_name<<"/"<<skill_name<<"/"<<param_name<<" change:");
        if (exploration_type == "variable_range")
        {
            int a;
            double tot, random_value;
            for (std::size_t i = 0; i < param_value.size(); i++)
            {
                a = i*3;
                tot = param_ratio.at(a) + param_ratio.at(a+1) + param_ratio.at(a+2);
                random_value = ((rand() % 101) * tot) / 100.0;

                if (random_value < param_ratio.at(a))
                {
                    new_param.at(i) = param_value.at(i) - ((exec_params_max_variation_[action_name][skill_name][param_name].at(i) * param_ratio.at(a) * (rand() % 100 + 1)) / 100.0);
                    new_param_test_number.at(a) = param_test_number.at(a) + 1;
                }
                else if ( random_value > (param_ratio.at(a)+param_ratio.at(a+1)) )
                {
                    new_param.at(i) = param_value.at(i) + ((exec_params_max_variation_[action_name][skill_name][param_name].at(i) * param_ratio.at(a) * (rand() % 100 + 1)) / 100.0);
                    new_param_test_number.at(a+2) = param_test_number.at(a+2) + 1;
                }
                else
                {
                    new_param.at(i) = param_value.at(i);
                    new_param_test_number.at(a+1) = param_test_number.at(a+1) + 1;
                }
                if (new_param.at(i) < exec_params_bound_[action_name][skill_name][param_name].at(i)[0])
                    new_param.at(i) = exec_params_bound_[action_name][skill_name][param_name].at(i)[0];
                if (new_param.at(i) > exec_params_bound_[action_name][skill_name][param_name].at(i)[1])
                    new_param.at(i) = exec_params_bound_[action_name][skill_name][param_name].at(i)[1];
            }

        }
        else
        {
            if (exploration_type != "standard")
                ROS_ERROR("Exploration type not found, use standard one");

            int a;
            double tot, random_value;
            for (std::size_t i = 0; i < param_value.size(); i++)
            {
                a = i*3;
                tot = param_ratio.at(a) + param_ratio.at(a+1) + param_ratio.at(a+2);
                random_value = ((rand() % 101) * tot) / 100.0;

                if (random_value < param_ratio.at(a))
                {
                    new_param.at(i) = param_value.at(i) - ((exec_params_max_variation_[action_name][skill_name][param_name].at(i) * (rand() % 100 + 1)) / 100.0);
                    new_param_test_number.at(a) = param_test_number.at(a) + 1;
                }
                else if ( random_value > (param_ratio.at(a)+param_ratio.at(a+1)) )
                {
                    new_param.at(i) = param_value.at(i) + ((exec_params_max_variation_[action_name][skill_name][param_name].at(i) * (rand() % 100 + 1)) / 100.0);
                    new_param_test_number.at(a+2) = param_test_number.at(a+2) + 1;
                }
                else
                {
                    new_param.at(i) = param_value.at(i);
                    new_param_test_number.at(a+1) = param_test_number.at(a+1) + 1;
                }
                if (new_param.at(i) < exec_params_bound_[action_name][skill_name][param_name].at(i)[0])
                    new_param.at(i) = exec_params_bound_[action_name][skill_name][param_name].at(i)[0];
                if (new_param.at(i) > exec_params_bound_[action_name][skill_name][param_name].at(i)[1])
                    new_param.at(i) = exec_params_bound_[action_name][skill_name][param_name].at(i)[1];
            }
        }

        if ( new_param.size()==1 )
        {
            setParam(action_name, skill_name, param_name, new_param.at(0));
        }
        else
        {
            setParam(action_name, skill_name, param_name, new_param);
        }
        printNewOldParam(param_name,new_param,param_value);
        printNewOldParam(name_test_number,new_param_test_number,param_test_number);
        setParam(action_name, skill_name, name_test_number, new_param_test_number);

    }

    test_number++;
    setParam(action_name, skill_name, "test_number", test_number);
    ROS_INFO_STREAM("/"<<action_name<<"/"<<skill_name<<"/test_number: "<<test_number);
    return skills_learning_msgs::SkillExploreResponse::Success;
}
//

//
int SkillsLearn::learning(const std::string &action_name, const std::string &skill_name, const std::string &learning_type)
{
    double reward, reward_old;
    std::vector<double> param, param_ratio, param_old, param_ratio_old;
    std::vector<int> param_test_number;
    int executed, test_number;
    std::string name_old, name_ratio, name_test_number;

    if ( !getParam(action_name, skill_name, "executed", executed))
    {
        ROS_WARN_STREAM("No param: "<<action_name<<"/"<<skill_name<<"/executed, set to 0");
        executed = 0;
        setParam(action_name, skill_name, "executed", executed);
    }

    if ( !getParam(action_name, skill_name, "test_number", test_number))
    {
        ROS_WARN_STREAM("No param: "<<action_name<<"/"<<skill_name<<"/test_number, set to 0");
        test_number = 0;
        setParam(action_name, skill_name, "test_number", test_number);
    }

    if (!executed)
    {
        ROS_GREEN_STREAM("/"<<action_name<<"/"<<skill_name<<" skill was not executed, the param return to the old ones");
        if ( test_number < 2)
        {
            ROS_GREEN_STREAM("The skill has not been performed more than once, the parameters cannot be reset");
            if ( test_number > 0)
            {
                setParam(action_name, skill_name, "test_number", test_number-1);
                ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<": "<<test_number<<"->"<<(test_number-1));
            }
        }
        else
        {
            for(const auto param_bound: exec_params_bound_[action_name][skill_name])
            {
                std::string param_name = param_bound.first;
                name_old = param_name;
                name_old.append("_old");
                name_test_number = param_name;
                name_test_number.append("_test_number");

                if ( !getParam(action_name, skill_name, name_old, param_old) )
                {
                    double d_param;
                    if ( !getParam(action_name, skill_name, name_old, d_param) )
                    {
                        ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_old<<" is not set");
                        return skills_learning_msgs::SkillLearningResponse::NoParam;
                    }
                    param_old.clear();
                    param_old.push_back(d_param);
                }
                if ( !getParam(action_name, skill_name, param_name, param) )
                {
                    double d_param;
                    if ( !getParam(action_name, skill_name, param_name, d_param) )
                    {
                        ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<param_name<<" is not set");
                        return skills_learning_msgs::SkillLearningResponse::NoParam;
                    }
                    param.clear();
                    param.push_back(d_param);
                }
                if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
                {
                    ROS_WARN_STREAM("The parameter /action_name/skill_name/name_test_number is not set");
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
                    setParam(action_name, skill_name, param_name, param_old.at(0));
                }
                else
                {
                    setParam(action_name, skill_name, param_name, param_old);
                }
                ROS_INFO_STREAM("/"<<action_name<<"/"<<skill_name<<"not executed: old_param->param");
                printNewOldParam(param_name,param,param_old);
                if ( test_number > 0)
                {
                    setParam(action_name, skill_name, "test_number", test_number-1);
                    ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<": "<<test_number<<"->"<<(test_number-1));
                }
            }
        }
        return skills_learning_msgs::SkillLearningResponse::Success;
    }
    else
    {
        setParam(action_name, skill_name, "executed", 0);
        ROS_INFO_STREAM("Set /" << action_name << "/" << skill_name << "/executed: " << 0 );
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

            for(const auto param_bound: exec_params_bound_[action_name][skill_name])
            {
                std::string param_name = param_bound.first;
                name_old = param_name;
                name_old.append("_old");


                if ( !getParam(action_name, skill_name, param_name, param) )
                {
                    double d_param;
                    if ( !getParam(action_name, skill_name, param_name, d_param) )
                    {
                        ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<param_name<<" is not set");
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
                printNewOldParam(param_name,param,param);
            }
            setParam(action_name, skill_name, "reward_old", initial_total_reward_);
            ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<initial_total_reward_);
            ROS_GREEN_STREAM("Learning finish");
            return skills_learning_msgs::SkillLearningResponse::Success;
        }
    }

    if ( !getParam(action_name, skill_name, "reward", reward) )
    {
        ROS_WARN_STREAM("The parameter /"<<action_name<<"/"<<skill_name<<"/reward is not set");
        reward = 0.0;
        setParam(action_name, skill_name, "reward", reward);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward);
    }
    ROS_INFO_STREAM("reward: "<<reward);

    if ( !getParam(action_name, skill_name, "reward_old", reward_old) )
    {
        ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/reward_old is not set");
        reward_old = initial_total_reward_;
        setParam(action_name, skill_name, "reward_old", reward_old);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<reward_old);
    }
    ROS_INFO_STREAM("reward_old: "<<reward_old);

    if ( total_reward_ > total_reward_old_ && reward >= reward_old)
    {
        ROS_INFO_STREAM("Total_reward > total_reward_old. Reward >= reward_old. Param is approved");
        ROS_INFO_STREAM(total_reward_<<" > "<<total_reward_old_<<". "<<reward<<" >= "<<reward_old<<".");
        setParam(action_name, skill_name, "reward_old", reward);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward_old: "<<reward);
    }
    else if ( total_reward_ > total_reward_old_ && reward < reward_old)
    {
        ROS_INFO_STREAM("Total_reward > total_reward_old. Reward < reward_old. Param not approved");
        ROS_INFO_STREAM(total_reward_<<" > "<<total_reward_old_<<". "<<reward<<" < "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }
    else if ( total_reward_ <= total_reward_old_ && reward >= reward_old)
    {
        ROS_INFO_STREAM("Total_reward <= total_reward_old. Reward >= reward_old. Param not approved");
        ROS_INFO_STREAM(total_reward_<<" <= "<<total_reward_old_<<". "<<reward<<" >= "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }
    else
    {
        ROS_INFO_STREAM("Total_reward <= total_reward_old. Reward < reward_old. Param not approved");
        ROS_INFO_STREAM(total_reward_<<" <= "<<total_reward_old_<<". "<<reward<<" < "<<reward_old<<".");
        setParam(action_name, skill_name, "reward", reward_old);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/reward: "<<reward_old);
    }

    for(const auto param_bound: exec_params_bound_[action_name][skill_name])
    {
        std::string param_name = param_bound.first;
        name_old = param_name;
        name_old.append("_old");
        name_ratio = param_name;
        name_ratio.append("_ratio");
        name_test_number = param_name;
        name_test_number.append("_test_number");

        if ( !getParam(action_name, skill_name, param_name, param) )
        {
            double d_param;
            if ( !getParam(action_name, skill_name, param_name, d_param) )
            {
                ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<param_name<<" is not set");
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
                ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_old<<" is not set");
                continue;
            }
            param_old.clear();
            param_old.push_back(d_param);
        }
        if ( !getParam(action_name, skill_name, name_ratio, param_ratio) )
        {
            ROS_WARN_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<name_ratio<<" is not set");
            continue;
        }
        param_ratio_old = param_ratio;

        if ( !getParam(action_name, skill_name, name_test_number, param_test_number) )
        {
            ROS_WARN_STREAM("The parameter /"<<action_name<<"/"<<skill_name<<"/"<<name_test_number<<" is not set");
            continue;
        }
        printArrayParam(name_test_number,param_test_number);

        printNewOldParam(param_name,param,param_old);

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
            ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_old);

            if (learning_type == "latest_more_weight")
            {
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
                if (learning_type != "standard")
                    ROS_ERROR("Learning type not found, use standard one");

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
        }
        else
        {
            if ( param_old.size() == 1 )
            {
                setParam(action_name, skill_name, param_name, param_old.at(0));
            }
            else
            {
                setParam(action_name, skill_name, param_name, param_old);
            }
            ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<param_name);

            if (learning_type == "latest_more_weight")
            {
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
            else
            {
                if (learning_type != "standard")
                    ROS_ERROR("Learning type not found, use standard one");

                for ( std::size_t i = 0; i < param.size(); i++ )
                {
                    int a = i*3;
                    if ( param.at(i) < param_old.at(i) )
                    {
                        if (param_test_number.at(a) == 0)
                        {
                            ROS_INFO_STREAM("No ");
                        }
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
        }
        printNewOldParam(param_name,param,param_old);
        printNewOldParam(name_ratio,param_ratio,param_ratio_old);

        setParam(action_name, skill_name, name_ratio, param_ratio);
        ROS_INFO_STREAM("Set /"<<action_name<<"/"<<skill_name<<"/"<<name_ratio);
    }
    return skills_learning_msgs::SkillLearningResponse::Success;
}
//

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
