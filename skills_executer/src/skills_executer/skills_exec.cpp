#include <skills_executer/skills_exec.h>

namespace skills_executer
{

SkillsExec::SkillsExec(const ros::NodeHandle &n, const std::string &name) : n_(n), robot_name_(name)
{
//    if (!n_.getParam("/skills_executer/use_ur", use_ur_))
//    {
//        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/use_ur param, defaul false");
//        use_ur_ = false;
//    }
//    if (!n_.getParam("/skills_executer/use_pybullet", use_pybullet_))
//    {
//        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/use_pybullet param, defaul false");
//        use_pybullet_ = false;
//    }
    if (!n_.getParam("/skills_executer/use_change_config_bridge", use_change_config_bridge_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/use_change_config_bridge param, defaul false");
        use_change_config_bridge_ = false;
    }
    if (!n_.getParam("/skills_executer/skills_parameters_name_space", param_ns_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skills_parameters_name_space param, defaul 'RL_params'");
        param_ns_ = "RL_params";
    }

    if (!n_.getParam("/skills_executer/robots/" + robot_name_ + "/end_link_frame",                       end_link_frame_))
    {
        ROS_ERROR_RED_STREAM("No /skills_executer/" + robot_name_ + "/end_link_frame param ");
        return;
    }
    if (!n_.getParam("/skills_executer/robots/" + robot_name_ + "/initial_reference_end_effector_frame", reference_end_effector_frame_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/" + robot_name_ + "/initial_reference_end_effector_frame param");
        reference_end_effector_frame_ = end_link_frame_;
    }
    if (!n_.getParam("/skills_executer/robots/" + robot_name_ + "/sensor_type",                          sensor_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/" + robot_name_ + "/sensor_type param");
    }
    else
    {
        if (!n_.getParam("/skills_executer/robots/" + robot_name_ + "/sensored_joint", sensored_joint_))
        {
            ROS_WARN_STREAM("No /skills_executer/" + robot_name_ + "/sensored_joint param");
        }
        else
        {
            std::string wrench_topic = "/" + robot_name_ + "/" + sensored_joint_ + "/wrench";
            wrench_sub_ = std::make_shared<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>>(n_, wrench_topic, 10);
        }
    }
    if (!n_.getParam("/skills_executer/robots/" + robot_name_ + "/initial_attached_link_name",           attached_link_name_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/" + robot_name_ + "/initial_attached_link_name param");
    }
    if (!n_.getParam("/skills_executer/robots/" + robot_name_ + "/initial_end_effector_touch_links",     end_effector_touch_links_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/" + robot_name_ + "/initial_end_effector_touch_links param");
    }

    if (!n_.getParam("/skills_executer/skill_types_names/watch_type",                         watch_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/watch_type param, defaul 'watch'");
        watch_type_ = "watch";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/cart_vel_type",                      cart_vel_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/cart_vel_type param, defaul 'cartesian_velocity'");
        cart_vel_type_ = "cartesian_velocity";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/cart_pos_type",                      cart_pos_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/cart_pos_type param, defaul 'cartesian_position'");
        cart_pos_type_ = "cartesian_position";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/cart_pos_to_type",                   cart_pos_to_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/cart_pos_to_type param, defaul 'cart_pos_to'");
        cart_pos_to_type_ = "cart_pos_to";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/simple_touch_type",                  simple_touch_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/simple_touch_type param, defaul 'simple_touch'");
        simple_touch_type_ = "simple_touch";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/parallel_2f_gripper_move_type",      parallel_2f_gripper_move_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/parallel_2f_gripper_move_type param, defaul 'parallel_gripper_move'");
        parallel_2f_gripper_move_type_ = "parallel_gripper_move";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/robotiq_gripper_move_type",          robotiq_gripper_move_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/robotiq_gripper_move_type param, defaul 'robotiq_gripper_move'");
        robotiq_gripper_move_type_ = "robotiq_gripper_move";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/ur_load_program_type",               ur_load_program_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/ur_load_program_type param, defaul 'ur_load_program'");
        ur_load_program_type_ = "ur_load_program";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/move_to_type",                       move_to_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/move_to_type param, defaul 'move_to'");
        move_to_type_ = "move_to";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/linear_move_type",                   linear_move_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/linear_move_type param, defaul 'linear_move'");
        linear_move_type_ = "linear_move";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/linear_move_to_type",                linear_move_to_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/linear_move_to_type param, defaul 'linear_move_to'");
        linear_move_to_type_ = "linear_move_to";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/joint_move_to_type",                 joint_move_to_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/joint_move_to_type param, defaul 'joint_move_to'");
        joint_move_to_type_ = "joint_move_to";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/release_end_effector_type",          release_end_effector_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/release_end_effector_type param, defaul 'release_end_effector'");
        release_end_effector_type_ = "release_end_effector";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/attach_end_effector_type",           attach_end_effector_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/attach_end_effector_type param, defaul 'attach_end_effector'");
        attach_end_effector_type_ = "attach_end_effector";
    }
    if (!n_.getParam("/skills_executer/skill_types_names/pneumatic_schunk_gripper_move_type", pneumatic_schunk_gripper_move_type_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/skill_types_names/pneumatic_schunk_gripper_move_type param, defaul 'pneumatic_schunk_gripper_move'");
        pneumatic_schunk_gripper_move_type_ = "pneumatic_schunk_gripper_move";
    }

    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("/" + robot_name_ + "/target_cart_twist",1);

    js_sub_ = std::make_shared<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>>(n_, "/joint_states", 10);

    if (use_change_config_bridge_)
    {
        change_config_clnt_ = n_.serviceClient<skills_util_msgs::ChangeConfig>("/skills_util/change_config");
        ROS_YELLOW_STREAM("Waiting for "<<change_config_clnt_.getService());
        change_config_clnt_.waitForExistence();
        ROS_YELLOW_STREAM("Connection ok");
    }
    else
    {
        change_config_clnt_ = n_.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
        ROS_YELLOW_STREAM("Waiting for "<<change_config_clnt_.getService());
        change_config_clnt_.waitForExistence();
        ROS_YELLOW_STREAM("Connection ok");
    }
    get_ik_clnt_ = n_.serviceClient<ik_solver_msgs::GetIk>("/" + robot_name_ + "/get_ik");
    ROS_YELLOW_STREAM("Waiting for "<<get_ik_clnt_.getService());
    get_ik_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");

//    if( ros::service::exists("/pybullet_sensor_reset") )
//    {
//        sensor_reset_clnt_ = n_.serviceClient<pybullet_simulation::SensorReset>("/pybullet_sensor_reset");
//        ROS_YELLOW_STREAM("Client to pybullet_sensor_reset service created");
//    }
//    else if( ros::service::exists("/another_sensor_topic") )
//    {
//        sensor_reset_clnt_ = n_.serviceClient<pybullet_simulation::SensorReset>("/another_sensor_server");
//        ROS_YELLOW_STREAM("Client to another_sensor_topic service created");
//    }
//    else
//    {
//        ROS_YELLOW_STREAM("No sensor server");
//    }

    touch_action_         = std::make_shared<actionlib::SimpleActionClient<simple_touch_controller_msgs::SimpleTouchAction>>("/" + robot_name_ + "/simple_touch", true);
    relative_move_action_ = std::make_shared<actionlib::SimpleActionClient<relative_cartesian_controller_msgs::RelativeMoveAction>>("/" + robot_name_ + "/relative_move", true);
    move_group_           = std::make_shared<moveit::planning_interface::MoveGroupInterface>(robot_name_); 

    tf::StampedTransform gripper_link_transform;
    try
    {
        tf_listener_.lookupTransform( reference_end_effector_frame_, end_link_frame_, ros::Time(0), gripper_link_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::transformTFToEigen( gripper_link_transform, T_gripper_to_end_link_);

    //    fjt part
    fjt_ac_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+robot_name_+"/follow_joint_trajectory",true));
    if (!n_.getParam("/skills_executer/" + robot_name_ + "/trajectory_joint_tollerance", trajectory_joint_tollerance_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/" + robot_name_ + "/trajectory_joint_tollerance param, defaul 0.001 for all joints");
        for ( std::size_t i = 0; i < move_group_->getJointNames().size(); i++)
        {
            trajectory_joint_tollerance_.push_back(0.001);
        }
    }
    else
    {
        if (trajectory_joint_tollerance_.size() != move_group_->getJointNames().size())
        {
            ROS_ERROR_RED_STREAM("Trajectory_joint_tollerance has wrong size, tjt: "<<trajectory_joint_tollerance_.size()<<", n_joint: "<<move_group_->getJointNames().size());
            return;
        }
    }
    if (!n_.getParam("/skills_executer/" + robot_name_ + "/trajectory_time_tollerance", trajectory_time_tollerance_))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/" + robot_name_ + "/trajectory_time_tollerance param, defaul 2 second");
        trajectory_time_tollerance_ = 2;
    }
    //    end

    double goal_joint_tollerance;
    if (!n_.getParam("/skills_executer/" + robot_name_ + "/goal_joint_tollerance", goal_joint_tollerance))
    {
        ROS_WARN_BOLDYELLOW_STREAM("No /skills_executer/" + robot_name_ + "/goal_joint_tollerance param, defaul 0.0001");
        move_group_->setGoalJointTolerance(0.0001);
    }
    else
    {
        move_group_->setGoalJointTolerance(goal_joint_tollerance);
    }

    skill_exec_srv_ = n_.advertiseService("/" + robot_name_ + "/skills_exec/execute_skill", &SkillsExec::skillsExecution, this);
}

bool SkillsExec::skillsExecution(skills_executer_msgs::SkillExecution::Request  &req,
                                 skills_executer_msgs::SkillExecution::Response &res)
{
    ROS_BOLDCYAN_STREAM("Skill requested: "<<req.action_name<<"/"<<req.skill_name);
    std::string skill_type;
    std::string action_type;
    std::string object_name;
    std::string location_name;
    int exec;
    double total_duration, total_max_force, final_distance;

    if (!getParam(req.action_name, "action_type",   action_type))
    {
        ROS_RED_STREAM("No param /"<<req.action_name<<"/action_type, skill execution finish");
        res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
        return true;
    }
    ROS_WHITE_STREAM("Action type: "<<action_type);

    if (!getParam(req.action_name, req.skill_name, "skill_type",   skill_type))
    {
        ROS_RED_STREAM("No param /"<<req.action_name<<"/"<<req.skill_name<<"/skill_type, skill execution finish");
        res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
        return true;
    }
    ROS_WHITE_STREAM("Skill type: "<<skill_type);

    if (!getParam(req.action_name, "executed", exec))
    {
        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/executed, considered equal to 0");
        exec = 0;
    }
    ROS_WHITE_STREAM("executed: "<<exec);
    if ( exec == 0)
    {
        setParam(req.action_name,"duration",0);
        setParam(req.action_name,"max_force",0);
    }

    if (!getParam(req.action_name, "duration", total_duration))
    {
        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/duration, considered equal to 0");
        total_duration = 0;
        setParam(req.action_name, "duration", total_duration);
    }
    if (!getParam(req.action_name, "max_force", total_max_force))
    {
        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/max_force, considered equal to 0");
        total_max_force = 0;
        setParam(req.action_name, "max_force", total_max_force);
    }

    setParam(req.action_name,"executed",1);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/executed: "<<1);

    if( !action_type.compare("pick_and_place") )
    {
        if (!getParam(req.action_name, "location_name", location_name))
        {
            ROS_RED_STREAM("No param /"<<req.action_name<<"/location_name, skill execution finish");
            res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
            return true;
        }
        ROS_WHITE_STREAM(req.action_name<<" location_name: "<<location_name);
        if (!getParam(req.action_name, "object_name", object_name))
        {
            ROS_RED_STREAM("No param /"<<req.action_name<<"/object_name, skill execution finish");
            res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
            return true;
        }
        ROS_WHITE_STREAM(req.action_name<<" object_name: "<<object_name);
    }

    max_force_ = 0.0;
    contact_ = false;
    end_force_thread_ = false;

    std::thread wrench_thread(&SkillsExec::maxWrenchCalculation, this);

    ros::Time initial_time = ros::Time::now();

    if ( !skill_type.compare(cart_vel_type_) )
    {
        res.result = cartVel(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(cart_pos_type_) )
    {
        res.result = cartPos(req.action_name, req.skill_name, 0);
    }
    else if ( !skill_type.compare(cart_pos_to_type_) )
    {
        res.result = cartPos(req.action_name, req.skill_name, 1);
    }
    else if ( !skill_type.compare(simple_touch_type_) )
    {
        res.result = simpleTouch(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(parallel_2f_gripper_move_type_) )
    {
        if (!object_name.empty())
        {
            current_grasped_object_ = object_name;
        }
        res.result = parallel2fGripperMove(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(robotiq_gripper_move_type_) )
    {
        res.result = robotiqGripperMove(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(ur_load_program_type_) )
    {
        res.result = urLoadProgram(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(move_to_type_) )
    {
        res.result = move_to(req.action_name, req.skill_name, 0);
    }
    else if ( !skill_type.compare(linear_move_to_type_) )
    {
        res.result = move_to(req.action_name, req.skill_name, 1);
    }
    else if ( !skill_type.compare(linear_move_type_) )
    {
        res.result = move_to(req.action_name, req.skill_name, 2);
    }
    else if ( !skill_type.compare(joint_move_to_type_) )
    {
        res.result = joint_move_to(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(pneumatic_schunk_gripper_move_type_) )
    {
        if (!object_name.empty())
        {
            current_grasped_object_ = object_name;
        }
        res.result = pneumaticSchunkGripperMove(req.action_name, req.skill_name);
    }
    else
    {
        ROS_RED_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<" result: NoSkillType");
        res.result = skills_executer_msgs::SkillExecutionResponse::NoSkillType;
        return true;
    }

    int int_result = res.result;
    ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<" result: "<<int_result);

    setParam(req.action_name,req.skill_name,"executed",1);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/executed: "<<1);
    if ( res.result != skills_executer_msgs::SkillExecutionResponse::Success )
    {
        setParam(req.action_name, "failure", 1);
        ROS_YELLOW_STREAM("Set /"<<req.action_name<<"/fail: "<<1);
    }

    double duration = ros::Time::now().toSec() - initial_time.toSec();
    total_duration = total_duration + duration;
    setParam(req.action_name,"duration",total_duration);

    end_force_thread_ = true;

    wrench_thread.join();

    end_force_thread_ = false;

    if ( max_force_ > total_max_force )
    {
        setParam(req.action_name, "max_force", max_force_);
    }

    setParam(req.action_name,req.skill_name,"duration",duration);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/duration: "<<duration);
    setParam(req.action_name,req.skill_name,"max_force",max_force_);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/max_force: "<<max_force_);
    setParam(req.action_name,req.skill_name,"contact",contact_);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/contact: "<<contact_);

    if( !action_type.compare("pick_and_place") )
    {
        if ( skill_type.find("gripper") == std::string::npos )
        {
            final_distance = tf_distance(object_name, location_name);
            double place_tollerance;
            ROS_WHITE_STREAM("Final distance: "<<final_distance);
            setParam(req.action_name,req.skill_name,"final_distance",final_distance);
            ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<"/final_distance: "<<final_distance);
            setParam(req.action_name,"final_distance",final_distance);
            ROS_WHITE_STREAM("Set /"<<req.action_name<<"/final_distance: "<<final_distance);
            if (!getParam(req.action_name, "place_tollerance", place_tollerance))
            {
                ROS_YELLOW_STREAM("No param: /"<<req.action_name<<"/place_tollerance, default: 0.01");
                place_tollerance = 0.01;
                setParam(req.action_name,"place_tollerance",place_tollerance);
                ROS_WHITE_STREAM("Set /"<<req.action_name<<"/place_tollerance: "<<place_tollerance);
            }
            ROS_WHITE_STREAM("Place_tollerance: "<<place_tollerance);
            if ( final_distance < place_tollerance )
            {
                ROS_WHITE_STREAM("Object in tollerance. Fail set to 0");
                setParam(req.action_name,"fail",0);
            }
            else
            {
                ROS_WHITE_STREAM("Object not in tollerance. Fail set to 1");
                setParam(req.action_name,"fail",1);
            }
        }
    }

    ROS_CYAN_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<" skill execution finish, return true");
    return true;
}

int SkillsExec::urLoadProgram(const std::string &action_name, const std::string &skill_name)
{
  std::string hw_name;
  if (not getParam(action_name, skill_name, "ur_hw_name", hw_name))
  {
      ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/ur_hw_name is not set");
      return skills_executer_msgs::SkillExecutionResponse::NoParam;
  }
  std::vector<std::string> programs;
  if (not getParam(action_name, skill_name, "programs", programs))
  {
      ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/program_name is not set");
      return skills_executer_msgs::SkillExecutionResponse::NoParam;
  }
  bool play;
  if (not getParam(action_name, skill_name, "play", play))
    play = true;

  ros::ServiceClient load_clnt = n_.serviceClient<ur_dashboard_msgs::Load>(hw_name+"/dashboard/load_program");
  ros::ServiceClient play_clnt = n_.serviceClient<std_srvs::Trigger>(hw_name+"/dashboard/play");

  ROS_YELLOW_STREAM("Waiting for "<<load_clnt.getService());
  load_clnt.waitForExistence();
  ROS_YELLOW_STREAM("Waiting for "<<play_clnt.getService());
  play_clnt.waitForExistence();

  ROS_YELLOW_STREAM("Connection ok");

  for(const std::string& program:programs)
  {
    ur_dashboard_msgs::Load load_srv;
    std_srvs::Trigger       play_srv;

    load_srv.request.filename = program;

    if(not load_clnt.call(load_srv) )
    {
        ROS_ERROR("Unable to load program: %s",load_srv.request.filename.c_str());
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
      if(not load_srv.response.success)
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ROS_YELLOW_STREAM("Program "<<load_srv.request.filename<<" loaded");

    if(play)
    {
      if(not play_clnt.call(play_srv) )
      {
          ROS_ERROR("Unable to play program: %s",load_srv.request.filename.c_str());
          return skills_executer_msgs::SkillExecutionResponse::Error;
      }
      else
      {
        if(not play_srv.response.success)
          return skills_executer_msgs::SkillExecutionResponse::Fail;
      }

      ROS_YELLOW_STREAM("Program "<<load_srv.request.filename<<" launched");
    }
  }

  return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::parallel2fGripperMove(const std::string &action_name, const std::string &skill_name)
{
    ROS_WHITE_STREAM("In parallel2fGripperMove");
    parallel_gripper_move_clnt_ = n_.serviceClient<parallel_2f_gripper::MoveGripper>("/move_parallel_gripper");
    ROS_WHITE_STREAM("Waiting for "<<parallel_gripper_move_clnt_.getService());
    if (!parallel_gripper_move_clnt_.waitForExistence(ros::Duration(2)))
    {
        ROS_ERROR_STREAM(parallel_gripper_move_clnt_.getService().c_str() << " don't connect.");
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    ROS_WHITE_STREAM("Connection ok");

    setParam(action_name,skill_name,"fail",0);
    ROS_WHITE_STREAM("Set "<<action_name<<"/"<<skill_name<<"/fail: 0");

    double torque;
    double velocity;
    double position;
    std::string control_mode;
    bool set = false;


    ROS_WHITE_STREAM("Read params");

    if (!getParam(action_name, skill_name, "control_mode", control_mode))
    {
        ROS_RED_STREAM("Control_mode not set");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    ROS_WHITE_STREAM("Control_mode: "<<control_mode);

    if (!control_mode.compare("torque"))
    {
        if (getParam(action_name, skill_name, "torque", torque))
        {
            ROS_WHITE_STREAM("Read torque: "<<torque);
            set = true;
        }
        else
        {
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<": torque parameter is missing. Return NoParam");
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        if (getParam(action_name, skill_name, "velocity", velocity))
        {
            ROS_WHITE_STREAM("Read velocity: "<<velocity);
        }
        else
        {
            velocity = 0;
        }
    }

    if (!control_mode.compare("velocity"))
    {
        if (getParam(action_name, skill_name, "velocity", velocity))
        {
            ROS_WHITE_STREAM("Read velocity: "<<velocity);
            set = true;
        }
        else
        {
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<": velocity parameter is missing. Return NoParam");
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        if (getParam(action_name, skill_name, "torque", torque))
        {
            ROS_WHITE_STREAM("Read torque: "<<torque);
        }
        else
        {
            torque = 0;
        }
    }

    if (!control_mode.compare("position"))
    {
        if (getParam(action_name, skill_name, "position", position))
        {
            ROS_WHITE_STREAM("Read position: "<<position);
            set = true;
        }
        else
        {
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<": position parameter is missing. Return NoParam");
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        if (getParam(action_name, skill_name, "torque", torque))
        {
            ROS_WHITE_STREAM("Read torque: "<<torque);
        }
        else
        {
            torque = 0;
        }
        if (getParam(action_name, skill_name, "velocity", velocity))
        {
            ROS_WHITE_STREAM("Read velocity: "<<velocity);
        }
        else
        {
            velocity = 0;
        }
    }

    if (!control_mode.compare("close"))
    {
        set = true;
        if (getParam(action_name, skill_name, "torque", torque))
        {
            ROS_WHITE_STREAM("Read torque: "<<torque);
        }
        else
        {
            torque = 0;
        }
        if (getParam(action_name, skill_name, "velocity", velocity))
        {
            ROS_WHITE_STREAM("Read velocity: "<<velocity);
        }
        else
        {
            velocity = 0;
        }
    }

    if (!control_mode.compare("open"))
    {
        set = true;
        if (getParam(action_name, skill_name, "torque", torque))
        {
            ROS_WHITE_STREAM("Read torque: "<<torque);
        }
        else
        {
            torque = 0;
        }
        if (getParam(action_name, skill_name, "velocity", velocity))
        {
            ROS_WHITE_STREAM("Read velocity: "<<velocity);
        }
        else
        {
            velocity = 0;
        }
    }

    if ( !set )
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<": some parameters are missing");
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_WHITE_STREAM(skill_name<<"-> torque: "<<torque);
    ROS_WHITE_STREAM(skill_name<<"-> velocity: "<<velocity);
    ROS_WHITE_STREAM(skill_name<<"-> position: "<<position);

    parallel_2f_gripper::MoveGripper move_gripper_srv;
    move_gripper_srv.request.control_mode = control_mode;
    move_gripper_srv.request.position     = position;
    move_gripper_srv.request.velocity     = velocity;
    move_gripper_srv.request.torque       = torque;

    bool closure;
    if (control_mode == "close")
    {
        closure = true;
    }
    else if ( (control_mode == "torque") & (torque > 0) )
    {
        closure = true;
    }
    else if ( (control_mode == "velocity") & (velocity > 0) )
    {
        closure = true;
    }
    else
    {
        closure = false;
    }

    if (!parallel_gripper_move_clnt_.call(move_gripper_srv))
    {
      ROS_RED_STREAM("Unable to call "<<parallel_gripper_move_clnt_.getService()<<" service ");
      return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ROS_WHITE_STREAM("Parallel_gripper_move return :"<<move_gripper_srv.response.result);
    if ( !move_gripper_srv.response.result.compare("true") )
    {
        sensor_msgs::JointState actual_js;
        if ( js_sub_->waitForANewData() )
        {
            actual_js = js_sub_->getData();
        }
        std::vector<std::string>::iterator index = std::find(actual_js.name.begin(), actual_js.name.end(), "right_finger_joint");

        if ( index !=  actual_js.name.end() )
        {
            if ( actual_js.position.at(index - actual_js.name.begin()) < parallel_2f_gripper_closed_position_ + gripper_tollerance_ && actual_js.position.at(index - actual_js.name.begin()) > parallel_2f_gripper_closed_position_ - gripper_tollerance_ )
            {
                setParam(action_name,skill_name,"fail",1);
                setParam(action_name,"fail",1);
                std::string param_name = "/" + current_grasped_object_ + "/attached";
                n_.setParam(param_name,false);
                ROS_YELLOW_STREAM("No object in the gripper. Current position: "<<actual_js.position.at(index - actual_js.name.begin()));
                return skills_executer_msgs::SkillExecutionResponse::Fail;
            }
        }

        std::string param_name = "/" + current_grasped_object_ + "/attached";
        if ( closure & !thread_esistence_ )
        {
            end_gripper_feedback_ = false;
            current_action_name_ = action_name;
            current_skill_name_  = skill_name;
            gripper_thread_ = std::make_shared<std::thread>([this]{gripper_feedback();});
            thread_esistence_ = true;
            std::string link_param_name = "/" + current_grasped_object_ + "/attached_link";
            std::string links_param_name = "/" + current_grasped_object_ + "/touch_links";
            n_.setParam(param_name,true);
            n_.setParam(link_param_name,attached_link_name_);
            n_.setParam(links_param_name,end_effector_touch_links_);
        }
        else
        {
            if ( thread_esistence_ )
            {
                end_gripper_feedback_ = true;

                if ( gripper_thread_->joinable() )
                {
                    gripper_thread_->join();
                    thread_esistence_ = false;
                }
            }
            n_.setParam(param_name, false);
        }
        end_gripper_feedback_ = false;

        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    else
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ros::Duration(1.0).sleep();

    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::pneumaticSchunkGripperMove(const std::string &action_name, const std::string &skill_name)
{
    ROS_WHITE_STREAM("In pneumaticSchunkGripperMove");
    std::string gripper_type;
    if (!getParam(action_name, skill_name, "gripper_type", gripper_type))
    {
        ROS_RED_STREAM("gripper_type param not set, default 'fake'");
        gripper_type = "fake";
    }

    if (gripper_type == "pybullet")
    {
        pneumatic_schunk_gripper_move_clnt_ = n_.serviceClient<std_srvs::SetBool>("/move_schunk_jpg_p_160_gripper");
        ROS_WHITE_STREAM("Waiting for "<<pneumatic_schunk_gripper_move_clnt_.getService());
        if (!pneumatic_schunk_gripper_move_clnt_.waitForExistence(ros::Duration(2)))
        {
            ROS_ERROR_STREAM(pneumatic_schunk_gripper_move_clnt_.getService().c_str() << " don't connect.");
            return skills_executer_msgs::SkillExecutionResponse::Fail;
        }
        ROS_WHITE_STREAM("Connection ok");
    }
    else if (gripper_type == "real")
    {
        pneumatic_schunk_gripper_move_clnt_ = n_.serviceClient<std_srvs::SetBool>("/battery_cell_utils/pneumatic_gripper");
        ROS_WHITE_STREAM("Waiting for "<<pneumatic_schunk_gripper_move_clnt_.getService());
        if (!pneumatic_schunk_gripper_move_clnt_.waitForExistence(ros::Duration(2)))
        {
            ROS_ERROR_STREAM(pneumatic_schunk_gripper_move_clnt_.getService().c_str() << " don't connect.");
            return skills_executer_msgs::SkillExecutionResponse::Fail;
        }
        ROS_WHITE_STREAM("Connection ok");
    }
    else if (gripper_type == "fake")
    {
//        add fake service
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    else
    {
        ROS_ERROR_STREAM("Wrong gripper type: "<< gripper_type);
        ROS_ERROR_STREAM("The possible values are: pybullet, real and fake");
    }

    setParam(action_name,skill_name,"fail",0);
    ROS_WHITE_STREAM("Set "<<action_name<<"/"<<skill_name<<"/fail: 0");

    std::string move;

    ROS_WHITE_STREAM("Read params");

    if (!getParam(action_name, skill_name, "movement", move))
    {
        ROS_RED_STREAM("movement param not set");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    std_srvs::SetBool move_srv;
    if (move == "open")
        move_srv.request.data = false;
    else if (move == "close")
        move_srv.request.data = true;

    std::string param_name = "/" + current_grasped_object_ + "/attached";

    if (!pneumatic_schunk_gripper_move_clnt_.call(move_srv))
    {
      ROS_RED_STREAM("Unable to call "<<pneumatic_schunk_gripper_move_clnt_.getService()<<" service ");
      return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ROS_WHITE_STREAM("pneumatic_schunk_gripper_move_clnt_ return :"<<move_srv.response.message);

    if (move_srv.request.data)
    {
        std::string link_param_name = "/" + current_grasped_object_ + "/attached_link";
        std::string links_param_name = "/" + current_grasped_object_ + "/touch_links";
        n_.setParam(param_name,true);
        n_.setParam(link_param_name,attached_link_name_);
        n_.setParam(links_param_name,end_effector_touch_links_);
    }
    else
    {
        n_.setParam(param_name,false);
    }

    ros::Duration(1.0).sleep();

    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::robotiqGripperMove(const std::string &action_name, const std::string &skill_name)
{
    ROS_WHITE_STREAM("In robotiqGripperMove");
    ros::ServiceClient gripper_clnt = n_.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");
    ROS_YELLOW_STREAM("Waiting for "<<gripper_clnt.getService() );
    if (!gripper_clnt.waitForExistence(ros::Duration(2)))
    {
        ROS_ERROR_STREAM(gripper_clnt.getService().c_str() << " don't connect.");
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    ROS_YELLOW_STREAM("Connection ok");
    manipulation_msgs::JobExecution gripper_srv;

    if (!getParam(action_name, skill_name, "property_id", gripper_srv.request.property_id))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/property_id is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "skill_name", gripper_srv.request.skill_name))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/skill_name is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "tool_id", gripper_srv.request.tool_id))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/tool_id is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    if ( !gripper_clnt.call(gripper_srv) )
    {
        ROS_ERROR("Unable to move gripper to %s state",gripper_srv.request.property_id.c_str());
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    if ( gripper_srv.response.results < 0 )
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    else
        return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::cartVel(const std::string &action_name, const std::string &skill_name)
{
    ROS_WHITE_STREAM("In cartVel");
    std::string        skill_type;
    std::string        frame_id;
    std::vector<float> twist_move;
    float              move_time;
    if (!getParam(action_name, skill_name, "skill_type", skill_type))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/skill_type is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "frame_id",   frame_id))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/frame_id is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "twist_move", twist_move))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/twist_move is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "move_time",  move_time))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/move_time is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_WHITE_STREAM(skill_name<<"-> frame_id: "<<frame_id.c_str());
    ROS_WHITE_STREAM(skill_name<<"-> twist_move: ["<<twist_move.at(0)<<","<<twist_move.at(1)<<","<<twist_move.at(2)<<","<<twist_move.at(3)<<","<<twist_move.at(4)<<","<<twist_move.at(5)<<"]");
    ROS_WHITE_STREAM(skill_name<<"-> move_time: "<<move_time);

    geometry_msgs::TwistStamped twist_command;

    twist_command.header.frame_id=frame_id;
    twist_command.twist.linear.x=twist_move.at(0);
    twist_command.twist.linear.y=twist_move.at(1);
    twist_command.twist.linear.z=twist_move.at(2);
    twist_command.twist.angular.x=twist_move.at(3);
    twist_command.twist.angular.y=twist_move.at(4);
    twist_command.twist.angular.z=twist_move.at(5);

    ROS_WHITE_STREAM("Change configuration: "<<skill_type);

    if ( !changeConfig(skill_type) )
    {
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    ROS_WHITE_STREAM("Execution Cart Move..");

    ros::Rate lp(100);

    double time = 0.0;

    while (time < move_time)
    {
      twist_command.header.stamp=ros::Time::now();
      twist_pub_.publish(twist_command);
      time = time+0.01;
      lp.sleep();
    }

    twist_command.twist.linear.x=0.0;
    twist_command.twist.linear.y=0.0;
    twist_command.twist.linear.z=0.0;
    twist_command.twist.angular.x=0.0;
    twist_command.twist.angular.y=0.0;
    twist_command.twist.angular.z=0.0;

    twist_command.header.stamp=ros::Time::now();
    twist_pub_.publish(twist_command);

    if ( !changeConfig(watch_type_) )
    {
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    if ( contact_ )
    {
        ROS_YELLOW_STREAM("Contact! /"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::cartPos(const std::string &action_name, const std::string &skill_name, const int &move_type)
{
    ROS_WHITE_STREAM("In cartPos");
    double rotZdeg, rotYdeg, rotXdeg, traXmm, traYmm, traZmm, target_angular_velocity, target_linear_velocity, vel;
    geometry_msgs::PoseStamped relative_pose;
    tf2::Quaternion quat;
    std::vector<double> position, orientation;
    relative_cartesian_controller_msgs::RelativeMoveGoal rel_move_goal;
    std::string skill_type;

    if ( !getParam(action_name, skill_name, "linear_velocity_m_s", target_linear_velocity) )
    {
        if ( !getParam(action_name, skill_name, "linear_velocity_mm_s", vel) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/linear_velocity_m_s or linear_velocity_mm_s is not set" );
            ROS_YELLOW_STREAM("The default value is linear_velocity_m_s = 0.250 m/s");
            target_linear_velocity = 0.250;
        }
        else
        {
            target_linear_velocity = vel / 1000;
            ROS_WHITE_STREAM("Read linear_velocity_mm_s: "<<vel);
        }
    }
    else
    {
        ROS_WHITE_STREAM("Read linear_velocity_m_s: "<<target_linear_velocity);
    }
    if ( !getParam(action_name, skill_name, "angular_velocity_rad_s", target_angular_velocity) )
    {
        if ( !getParam(action_name, skill_name, "angular_velocity_deg_s", vel) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/angular_velocity_rad_s or angular_velocity_deg_s is not set" );
            ROS_YELLOW_STREAM("The default value is angular_velocity_deg_s = 30 deg/sec");
            target_angular_velocity = 30*M_PI/180;
        }
        else
        {
            target_angular_velocity = vel*M_PI/180;
            ROS_WHITE_STREAM("Read angular_velocity_deg_s: "<<vel);
            ROS_WHITE_STREAM("Angular_velocity_rad_s: "<<target_angular_velocity);
        }
    }
    else
    {
        ROS_WHITE_STREAM("Read angular_velocity_rad_s: "<<target_angular_velocity);
    }
    if ( !getParam(action_name, skill_name, "skill_type", skill_type) )
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/skill_type is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    if ( move_type == 0)
    {
        if ( !getParam(action_name, skill_name, "frame", relative_pose.header.frame_id) )
        {
            ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/frame is not set" );
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        if ( getParam(action_name, skill_name, "rotZdeg", rotZdeg) )
        {
            double angle = rotZdeg*M_PI/180;
            quat.setRPY(0,0,angle);
            //        relative_pose.pose.orientation=tf::createQuaternionFromRPY(0.0,0.0,angle);
            relative_pose.pose.orientation.x = quat.getX();
            relative_pose.pose.orientation.y = quat.getY();
            relative_pose.pose.orientation.z = quat.getZ();
            relative_pose.pose.orientation.w = quat.getW();
            relative_pose.pose.position.x = 0.0;
            relative_pose.pose.position.y = 0.0;
            relative_pose.pose.position.z = 0.0;
            ROS_WHITE_STREAM("Read rotZdeg: "<<rotZdeg);
            ROS_WHITE_STREAM("Position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
            ROS_WHITE_STREAM("Orientation: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
        }
        else if( getParam(action_name, skill_name, "rotYdeg", rotYdeg) )
        {
            double angle = rotYdeg*M_PI/180;
            quat.setRPY(0,angle,0);
            //        relative_pose.pose.orientation=tf::createQuaternionFromRPY(0.0,angle,0.0);
            relative_pose.pose.orientation.x = quat.getX();
            relative_pose.pose.orientation.y = quat.getY();
            relative_pose.pose.orientation.z = quat.getZ();
            relative_pose.pose.orientation.w = quat.getW();
            relative_pose.pose.position.x = 0.0;
            relative_pose.pose.position.y = 0.0;
            relative_pose.pose.position.z = 0.0;
            ROS_WHITE_STREAM("Read rotYdeg: "<<rotYdeg);
            ROS_WHITE_STREAM("Position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
            ROS_WHITE_STREAM("Orientation: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
        }
        else if( getParam(action_name, skill_name, "rotXdeg", rotXdeg) )
        {
            double angle = rotXdeg*M_PI/180;
            quat.setRPY(angle,0,0);
            //        relative_pose.pose.orientation=tf::createQuaternionFromRPY(angle,0.0,0.0);
            relative_pose.pose.orientation.x = quat.getX();
            relative_pose.pose.orientation.y = quat.getY();
            relative_pose.pose.orientation.z = quat.getZ();
            relative_pose.pose.orientation.w = quat.getW();
            relative_pose.pose.position.x = 0.0;
            relative_pose.pose.position.y = 0.0;
            relative_pose.pose.position.z = 0.0;
            ROS_WHITE_STREAM("Read rotXdeg: "<<rotXdeg);
            ROS_WHITE_STREAM("Position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
            ROS_WHITE_STREAM("Orientation: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
        }
        else if( getParam(action_name, skill_name, "traXmm", traXmm) )
        {
            relative_pose.pose.position.x = traXmm/1000;
            relative_pose.pose.position.y = 0.0;
            relative_pose.pose.position.z = 0.0;
            relative_pose.pose.orientation.x = 0.0;
            relative_pose.pose.orientation.y = 0.0;
            relative_pose.pose.orientation.z = 0.0;
            relative_pose.pose.orientation.w = 1.0;
            ROS_WHITE_STREAM("Read traXmm: "<<traXmm);
            ROS_WHITE_STREAM("Position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
            ROS_WHITE_STREAM("Orientation: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
        }
        else if( getParam(action_name, skill_name, "traYmm", traYmm) )
        {
            relative_pose.pose.position.x = 0.0;
            relative_pose.pose.position.y = traYmm/1000;
            relative_pose.pose.position.z = 0.0;
            relative_pose.pose.orientation.x = 0.0;
            relative_pose.pose.orientation.y = 0.0;
            relative_pose.pose.orientation.z = 0.0;
            relative_pose.pose.orientation.w = 1.0;
            ROS_WHITE_STREAM("Read traYmm: "<<traYmm);
            ROS_WHITE_STREAM("Position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
            ROS_WHITE_STREAM("Orientation: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
        }
        else if( getParam(action_name, skill_name, "traZmm", traZmm) )
        {
            relative_pose.pose.position.x = 0.0;
            relative_pose.pose.position.y = 0.0;
            relative_pose.pose.position.z = traZmm/1000;
            relative_pose.pose.orientation.x = 0.0;
            relative_pose.pose.orientation.y = 0.0;
            relative_pose.pose.orientation.z = 0.0;
            relative_pose.pose.orientation.w = 1.0;
            ROS_WHITE_STREAM("Read traZmm: "<<traZmm);
            ROS_WHITE_STREAM("Position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
            ROS_WHITE_STREAM("Orientation: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
        }
        else
        {
            if ( !getParam(action_name, skill_name, "position", position) )
            {
                ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/position is not set" );
                ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            else
            {
                if ( position.size() != 3 )
                {
                    ROS_RED_STREAM("The position size is not 3");
                    ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                    return skills_executer_msgs::SkillExecutionResponse::NoParam;
                }
                relative_pose.pose.position.x = position.at(0);
                relative_pose.pose.position.y = position.at(1);
                relative_pose.pose.position.z = position.at(2);
                ROS_WHITE_STREAM("Read position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
            }
            if ( !getParam(action_name, skill_name, "quaternion", orientation) )
            {
                ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/quaternion is not set" );
                ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            else
            {
                if ( orientation.size() != 4 )
                {
                    ROS_RED_STREAM("The quaternion size is not 4");
                    ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                    return skills_executer_msgs::SkillExecutionResponse::NoParam;
                }
                relative_pose.pose.orientation.x = orientation.at(0);
                relative_pose.pose.orientation.y = orientation.at(1);
                relative_pose.pose.orientation.z = orientation.at(2);
                relative_pose.pose.orientation.w = orientation.at(3);
                ROS_WHITE_STREAM("Read quaternion: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
            }
        }

        rel_move_goal.target_angular_velocity = target_angular_velocity;
        rel_move_goal.target_linear_velocity = target_linear_velocity;
        rel_move_goal.relative_pose = relative_pose;

        ROS_WHITE_STREAM("Goal:");
        ROS_WHITE_STREAM("Frame: "<<rel_move_goal.relative_pose.header.frame_id);
        ROS_WHITE_STREAM("Position: ["<<rel_move_goal.relative_pose.pose.position.x<<","<<rel_move_goal.relative_pose.pose.position.y<<","<<rel_move_goal.relative_pose.pose.position.z<<"]");
        ROS_WHITE_STREAM("Quaternion: ["<<rel_move_goal.relative_pose.pose.orientation.x<<","<<rel_move_goal.relative_pose.pose.orientation.y<<","<<rel_move_goal.relative_pose.pose.orientation.z<<","<<rel_move_goal.relative_pose.pose.orientation.w<<"]");
        ROS_WHITE_STREAM("Velocity: lin "<<rel_move_goal.target_linear_velocity<<", rot "<<rel_move_goal.target_angular_velocity);
    }
    if ( move_type == 1)
    {
        std::string target_TF;

        if (!getParam(action_name, skill_name, "target_frame", target_TF))
        {
            ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/target_TF is not set" );
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }

        tf::StampedTransform gripper_to_goal_frame_transform;
        try
        {
            tf_listener_.lookupTransform( reference_end_effector_frame_, target_TF, ros::Time(0), gripper_to_goal_frame_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ROS_WHITE_STREAM("  target_frame: "<<target_TF);
        Eigen::Affine3d T_gripper_to_goal_frame;
        tf::transformTFToEigen( gripper_to_goal_frame_transform, T_gripper_to_goal_frame);

        std::vector<double> relative_position, relative_quaternion;
        tf::Vector3 rel_pos;
        tf::Quaternion rel_quat;
        tf::StampedTransform relative_transform;
        Eigen::Affine3d T_relative;
        if ( !getParam(action_name, skill_name, "relative_position", relative_position) )
        {
            ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/relative_position is not set. It is considered [0,0,0]" );
            relative_position.clear();
            relative_position.push_back(0);
            relative_position.push_back(0);
            relative_position.push_back(0);
            setParam(action_name, skill_name, "relative_position", relative_position);
            rel_pos.setX(0);
            rel_pos.setY(0);
            rel_pos.setZ(0);
        }
        else
        {
            if ( relative_position.size() != 3 )
            {
                ROS_YELLOW_STREAM("The relative_position size is not 3");
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_position is considered [0,0,0]" );
                relative_position.push_back(0);
                relative_position.push_back(0);
                relative_position.push_back(0);
            }
            else
            {
                rel_pos.setX(relative_position.at(0));
                rel_pos.setY(relative_position.at(1));
                rel_pos.setZ(relative_position.at(2));
                ROS_WHITE_STREAM("  relative_position: ["<<rel_pos.getX()<<","<<rel_pos.getY()<<","<<rel_pos.getZ()<<"]");
            }
        }
        if ( !getParam(action_name, skill_name, "relative_orientation", relative_quaternion) )
        {
            ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/relative_orientation is not set. It is considered [0,0,0,1]" );
            relative_quaternion.clear();
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            setParam(action_name, skill_name, "relative_orientation", relative_quaternion);
            rel_quat.setX(0);
            rel_quat.setY(0);
            rel_quat.setZ(0);
            rel_quat.setW(1);
        }
        else
        {
            if ( relative_quaternion.size() != 4 )
            {
                ROS_YELLOW_STREAM("The relative_quaternion size is not 4");
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_orientation is considered [0,0,0,1]" );
                relative_quaternion.push_back(0);
                relative_quaternion.push_back(0);
                relative_quaternion.push_back(0);
                relative_quaternion.push_back(1);
            }
            rel_quat.setX(relative_quaternion.at(0));
            rel_quat.setY(relative_quaternion.at(1));
            rel_quat.setZ(relative_quaternion.at(2));
            rel_quat.setW(relative_quaternion.at(3));
            ROS_WHITE_STREAM("  relative_orientation: ["<<rel_quat.getX()<<","<<rel_quat.getY()<<","<<rel_quat.getZ()<<","<<rel_quat.getW()<<"]");
        }
        relative_transform.setOrigin(rel_pos);
        relative_transform.setRotation(rel_quat);
        tf::transformTFToEigen(relative_transform, T_relative);

        tf::StampedTransform gripper_to_real_goal_transform;
        Eigen::Affine3d T_gripper_to_real_goal = T_gripper_to_goal_frame* T_relative;
        tf::transformEigenToTF(T_gripper_to_real_goal, gripper_to_real_goal_transform);
        relative_pose.pose.position.x    = gripper_to_real_goal_transform.getOrigin().getX();
        relative_pose.pose.position.y    = gripper_to_real_goal_transform.getOrigin().getY();
        relative_pose.pose.position.z    = gripper_to_real_goal_transform.getOrigin().getZ();
        relative_pose.pose.orientation.x = gripper_to_real_goal_transform.getRotation().getX();
        relative_pose.pose.orientation.y = gripper_to_real_goal_transform.getRotation().getY();
        relative_pose.pose.orientation.z = gripper_to_real_goal_transform.getRotation().getZ();
        relative_pose.pose.orientation.w = gripper_to_real_goal_transform.getRotation().getW();
        relative_pose.header.frame_id    = reference_end_effector_frame_;

        rel_move_goal.target_angular_velocity = target_angular_velocity;
        rel_move_goal.target_linear_velocity = target_linear_velocity;
        rel_move_goal.relative_pose = relative_pose;

        ROS_WHITE_STREAM("Goal:");
        ROS_WHITE_STREAM("Frame: "<<rel_move_goal.relative_pose.header.frame_id);
        ROS_WHITE_STREAM("Position: ["<<rel_move_goal.relative_pose.pose.position.x<<","<<rel_move_goal.relative_pose.pose.position.y<<","<<rel_move_goal.relative_pose.pose.position.z<<"]");
        ROS_WHITE_STREAM("Quaternion: ["<<rel_move_goal.relative_pose.pose.orientation.x<<","<<rel_move_goal.relative_pose.pose.orientation.y<<","<<rel_move_goal.relative_pose.pose.orientation.z<<","<<rel_move_goal.relative_pose.pose.orientation.w<<"]");
        ROS_WHITE_STREAM("Velocity: lin "<<rel_move_goal.target_linear_velocity<<", rot "<<rel_move_goal.target_angular_velocity);
    }

    if ( !changeConfig(skill_type) )
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return ProblemConfManager: "<<skills_executer_msgs::SkillExecutionResponse::ProblemConfManager);
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    relative_move_action_->waitForServer();
    relative_move_action_->sendGoalAndWait(rel_move_goal);

    if ( !changeConfig(watch_type_) )
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return ProblemConfManager: "<<skills_executer_msgs::SkillExecutionResponse::ProblemConfManager);
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    if ( contact_ )
    {
        ROS_RED_STREAM("Contact! /"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    if ( relative_move_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    else
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
}

int SkillsExec::simpleTouch(const std::string &action_name, const std::string &skill_name)
{
    ROS_WHITE_STREAM("In simpleTouch");
    std::string         skill_type;
    std::string         goal_twist_frame;
    std::vector<double> goal_twist;

    double target_force;
    bool   relative_target;
    double release;
    int    release_condition;

    if (!getParam(action_name, skill_name, "skill_type", skill_type) )
    {
        ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/skill_type is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> skill_type: "<<skill_type);
    if (!getParam(action_name, skill_name, "goal_twist_frame", goal_twist_frame))
    {
        ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/goal_twist_frame is not set");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> goal_twist_frame: "<<goal_twist_frame);
    if (!getParam(action_name, skill_name, "goal_twist", goal_twist))
    {
        double d_g_t;
        if (!getParam(action_name, skill_name, "goal_twist", d_g_t))
        {
            ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/goal_twist is not set" );
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        goal_twist.clear();
        goal_twist.push_back(d_g_t);
    }
    if ( goal_twist.size() == 1)
    {
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> goal_twist: "<<goal_twist.at(0));
    }
    else if ( goal_twist.size() == 6 )
    {
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> goal_twist: ["<<goal_twist.at(0)<<","<<goal_twist.at(1)<<","<<goal_twist.at(2)<<","<<goal_twist.at(3)<<","<<goal_twist.at(4)<<","<<goal_twist.at(5)<<"]");
    }
    else
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<"-> goal_twist has wrong size");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    if (!getParam(action_name, skill_name, "target_force", target_force))
    {
        if (!getParam(action_name, skill_name, "target_wrench", target_force))
        {
            ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/target_wrench or target_force are not set" );
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
    }
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> target_wrench: "<<target_force);

    if (!getParam(action_name, skill_name, "release", release))
    {
        ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/release is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    else
    {
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> release: "<<release);
    }

    if (!getParam(action_name, skill_name, "release_condition", release_condition))
    {
        ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/release_condition is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    else
    {
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> release_condition: "<<release_condition);
    }

    if (!getParam(action_name, skill_name, "relative_target", relative_target))
    {
        ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_target is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (relative_target)
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> relative_target: true");
    else
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> relative_target: false");

    ROS_WHITE_STREAM("Change configuration: "<<skill_type );

    if ( !changeConfig(skill_type) )
    {
        ROS_RED_STREAM("Problem with configuration manager" );
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    ROS_WHITE_STREAM("Execution Simple Touch..");

    simple_touch_controller_msgs::SimpleTouchGoal goal_touch;

    goal_touch.goal_twist = goal_twist;
    goal_touch.goal_twist_frame = goal_twist_frame;
    goal_touch.relative_target = relative_target;
    goal_touch.release = release;
    goal_touch.release_condition = release_condition;
    goal_touch.target_force = target_force;

    touch_action_->waitForServer();
    touch_action_->sendGoalAndWait(goal_touch);

    ROS_WHITE_STREAM("Simple Touch finisched");

    if ( !changeConfig(watch_type_) )
    {
        ROS_RED_STREAM("Problem with configuration manager" );
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    if ( touch_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WHITE_STREAM("Simple Touch success");
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    else
    {
        ROS_RED_STREAM("Simple touch failed" );
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
}

int SkillsExec::move_to(const std::string &action_name, const std::string &skill_name, const int &move_type)
{
    ROS_WHITE_STREAM("In move_to");
    if ( !changeConfig("trajectory_tracking") )
    {
        ROS_RED_STREAM("Problem with configuration manager");
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    if ( !fjt_ac_->waitForServer(ros::Duration(10)) )
    {
        ROS_RED_STREAM("Timeout FollowJointTrajectory client for robot "<<robot_name_);
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    ros::Duration(0.5).sleep();
    ROS_WHITE_STREAM("Move info: ");
    std::string target_TF;
    double acc, vel, p_time;
    int r_att;

    if (!getParam(action_name, skill_name, "acceleration_scaling", acc))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/acceleration_scaling is not set, defaul value: 0.5" );
        acc = 0.5;
        setParam(action_name, skill_name, "acceleration_scaling", acc);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/acceleration_scaling: "<<acc);
    }
    move_group_->setMaxAccelerationScalingFactor(acc);
    ROS_WHITE_STREAM("  acceleration_scaling: "<<acc);

    if (!getParam(action_name, skill_name, "velocity_scaling", vel))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/velocity_scaling is not set, defaul value: 0.5" );
        vel = 0.5;
        setParam(action_name, skill_name, "velocity_scaling", vel);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/velocity_scaling: "<<vel);
    }
    move_group_->setMaxVelocityScalingFactor(vel);
    ROS_WHITE_STREAM("  velocity_scaling: "<<vel);

    if (!getParam(action_name, skill_name, "planning_time", p_time))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/planning_time is not set, defaul value: 5.0" );
        p_time = 0.5;
        setParam(action_name, skill_name, "planning_time", p_time);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/planning_time: "<<p_time);
    }
    move_group_->setPlanningTime(p_time);
    ROS_WHITE_STREAM("  planning_time: "<<p_time);

    if (!getParam(action_name, skill_name, "replan_attempts", r_att))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/replan_attempts is not set, defaul value: 10" );
        r_att = 10;
        setParam(action_name, skill_name, "replan_attempts", r_att);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/replan_attempts: "<<r_att);
    }
    move_group_->setReplanAttempts(r_att);
    ROS_WHITE_STREAM("  replan_attempts: "<<r_att);

    if ( move_type < 2)
    {
        tf::StampedTransform origin_goal_transform, origin_link_goal_transform;
        std::vector<double> relative_position, relative_quaternion;
        tf::Vector3 rel_pos;
        tf::Quaternion rel_quat;
        tf::StampedTransform relative_transform;
        Eigen::Affine3d T_relative;

        if (!getParam(action_name, skill_name, "target_frame", target_TF))
        {
            ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/target_TF is not set" );
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }

        try
        {
            tf_listener_.lookupTransform( "world", target_TF, ros::Time(0), origin_goal_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ROS_WHITE_STREAM("  target_frame: "<<target_TF);

        if ( !getParam(action_name, skill_name, "relative_position", relative_position) )
        {
            ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/relative_position is not set. It is considered [0,0,0]" );
            relative_position.clear();
            relative_position.push_back(0);
            relative_position.push_back(0);
            relative_position.push_back(0);
            setParam(action_name, skill_name, "relative_position", relative_position);
            rel_pos.setX(0);
            rel_pos.setY(0);
            rel_pos.setZ(0);
        }
        else
        {
            if ( relative_position.size() != 3 )
            {
                ROS_YELLOW_STREAM("The relative_position size is not 3");
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_position is considered [0,0,0]" );
                relative_position.clear();
                relative_position.push_back(0);
                relative_position.push_back(0);
                relative_position.push_back(0);
                setParam(action_name, skill_name, "relative_position", relative_position);
                rel_pos.setX(0);
                rel_pos.setY(0);
                rel_pos.setZ(0);
            }
            else
            {
                rel_pos.setX(relative_position.at(0));
                rel_pos.setY(relative_position.at(1));
                rel_pos.setZ(relative_position.at(2));
                ROS_WHITE_STREAM("  relative_position: ["<<rel_pos.getX()<<","<<rel_pos.getY()<<","<<rel_pos.getZ()<<"]");
            }
        }
        if ( !getParam(action_name, skill_name, "relative_orientation", relative_quaternion) )
        {
            ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/relative_orientation is not set. It is considered [0,0,0,1]" );
            relative_quaternion.clear();
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            setParam(action_name, skill_name, "relative_orientation", relative_quaternion);
            rel_quat.setX(0);
            rel_quat.setY(0);
            rel_quat.setZ(0);
            rel_quat.setW(1);
        }
        else
        {
            if ( relative_quaternion.size() != 4 )
            {
                ROS_YELLOW_STREAM("The relative_quaternion size is not 4");
                ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_orientation is considered [0,0,0,1]" );
                relative_quaternion.clear();
                relative_quaternion.push_back(0);
                relative_quaternion.push_back(0);
                relative_quaternion.push_back(0);
                relative_quaternion.push_back(1);
                setParam(action_name, skill_name, "relative_orientation", relative_quaternion);
                rel_quat.setX(0);
                rel_quat.setY(0);
                rel_quat.setZ(0);
                rel_quat.setW(1);
            }
            else
            {
                rel_quat.setX(relative_quaternion.at(0));
                rel_quat.setY(relative_quaternion.at(1));
                rel_quat.setZ(relative_quaternion.at(2));
                rel_quat.setW(relative_quaternion.at(3));
                ROS_WHITE_STREAM("  relative_orientation: ["<<rel_quat.getX()<<","<<rel_quat.getY()<<","<<rel_quat.getZ()<<","<<rel_quat.getW()<<"]");
            }
        }
        relative_transform.setOrigin(rel_pos);
        relative_transform.setRotation(rel_quat);
        tf::transformTFToEigen(relative_transform, T_relative);

        Eigen::Affine3d T_origin_goal_gripper;
        tf::transformTFToEigen( origin_goal_transform, T_origin_goal_gripper);
        Eigen::Affine3d T_origin_link_goal = T_origin_goal_gripper* T_relative * T_gripper_to_end_link_;
        tf::transformEigenToTF(T_origin_link_goal,origin_link_goal_transform);

        geometry_msgs::Pose target_pose;

        target_pose.position.x    = origin_link_goal_transform.getOrigin().getX();
        target_pose.position.y    = origin_link_goal_transform.getOrigin().getY();
        target_pose.position.z    = origin_link_goal_transform.getOrigin().getZ();
        target_pose.orientation.x = origin_link_goal_transform.getRotation().getX();
        target_pose.orientation.y = origin_link_goal_transform.getRotation().getY();
        target_pose.orientation.z = origin_link_goal_transform.getRotation().getZ();
        target_pose.orientation.w = origin_link_goal_transform.getRotation().getW();

        if ( move_type == 1)
        {
            geometry_msgs::Pose actual_pose = move_group_->getCurrentPose().pose;
            std::vector<geometry_msgs::Pose> waypoints;

            waypoints.push_back(actual_pose);
            waypoints.push_back(target_pose);
            moveit::core::RobotStatePtr robot_state_ptr = move_group_->getCurrentState();
            moveit_msgs::RobotState robot_state;
            moveit::core::robotStateToRobotStateMsg(*robot_state_ptr, robot_state);
            moveit_msgs::RobotTrajectory moveit_trj;
            double plan_time = ros::Time::now().toSec();

            if ( move_group_->computeCartesianPath(waypoints,0.001,0.0,moveit_trj) < 0.0 )
            {
                ROS_RED_STREAM("Linear_plan_result: unknown error");
                ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
                return skills_executer_msgs::SkillExecutionResponse::Fail;
            }
            plan_time = ros::Time::now().toSec() - plan_time;
            moveit_plan_.planning_time_ = plan_time;
            moveit_plan_.start_state_   = robot_state;
            moveit_plan_.trajectory_    = moveit_trj;
        }
        if ( move_type == 0)
        {
            move_group_->setPoseTarget(target_pose);
            moveit::core::MoveItErrorCode plan_result = move_group_->plan(moveit_plan_);
            ROS_WHITE_STREAM("Plan_result: "<<plan_result);
            if ( plan_result != moveit::core::MoveItErrorCode::SUCCESS )
            {
                ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
                if ( !changeConfig(watch_type_) )
                {
                    ROS_RED_STREAM("Problem with configuration manager");
                    return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
                }
                return skills_executer_msgs::SkillExecutionResponse::Fail;
            }
        }
    }
    else if ( move_type == 2 )
    {
        tf::StampedTransform origin_link_goal_transform;

        std::string move_reference_frame;
        geometry_msgs::Pose actual_pose = move_group_->getCurrentPose().pose;
        geometry_msgs::Pose target_pose;
        std::vector<geometry_msgs::Pose> waypoints;
        std::vector<double> position, orientation;
        tf::Vector3 pos;
        tf::Quaternion quat;
        tf::StampedTransform movement_transform, link_to_reference_transform, reference_to_link_transform, link_to_reference_rotation, reference_to_link_rotation, origin_to_link_transform;

        if ( !getParam(action_name, skill_name, "position", position) )
        {
            ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/position is not set" );
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( position.size() != 3 )
            {
                ROS_RED_STREAM("The position size is not 3");
                ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            pos.setX(position.at(0));
            pos.setY(position.at(1));
            pos.setZ(position.at(2));
            ROS_WHITE_STREAM("Read position: ["<<pos.getX()<<","<<pos.getY()<<","<<pos.getZ()<<"]");
        }
        if ( !getParam(action_name, skill_name, "quaternion", orientation) )
        {
            ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/quaternion is not set" );
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( orientation.size() != 4 )
            {
                ROS_RED_STREAM("The quaternion size is not 4");
                ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            quat.setX(orientation.at(0));
            quat.setY(orientation.at(1));
            quat.setZ(orientation.at(2));
            quat.setW(orientation.at(3));
            ROS_WHITE_STREAM("Read quaternion: ["<<quat.getX()<<","<<quat.getY()<<","<<quat.getZ()<<","<<quat.getW()<<"]");
       }
        movement_transform.setOrigin(pos);
        movement_transform.setRotation(quat);

        if ( !getParam(action_name, skill_name, "frame", move_reference_frame) )
        {
            ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/frame is not set" );
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }

        ROS_WHITE_STREAM("Read frame: "<<move_reference_frame);

        try
        {
            tf_listener_.lookupTransform( end_link_frame_, move_reference_frame, ros::Time(0), link_to_reference_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        try
        {
            tf_listener_.lookupTransform( move_reference_frame, end_link_frame_, ros::Time(0), reference_to_link_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        tf::Vector3 null_vector;
        null_vector.setX(0);
        null_vector.setY(0);
        null_vector.setZ(0);
        link_to_reference_rotation = link_to_reference_transform;
        link_to_reference_rotation.setOrigin( null_vector );
        reference_to_link_rotation = reference_to_link_transform;
        reference_to_link_rotation.setOrigin( null_vector );

        try
        {
            tf_listener_.lookupTransform( "world", end_link_frame_, ros::Time(0), origin_to_link_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        Eigen::Affine3d T_origin_to_link, R_link_to_reference, T_movement, R_reference_to_link;

        tf::transformTFToEigen( origin_to_link_transform, T_origin_to_link);
        tf::transformTFToEigen( link_to_reference_rotation, R_link_to_reference);
        tf::transformTFToEigen( movement_transform, T_movement);
        tf::transformTFToEigen( reference_to_link_rotation, R_reference_to_link);

        Eigen::Affine3d T_origin_link_goal = T_origin_to_link * R_link_to_reference * T_movement * R_reference_to_link;

        tf::transformEigenToTF(T_origin_link_goal,origin_link_goal_transform);

        target_pose.position.x = origin_link_goal_transform.getOrigin().getX();
        target_pose.position.y = origin_link_goal_transform.getOrigin().getY();
        target_pose.position.z = origin_link_goal_transform.getOrigin().getZ();
        target_pose.orientation.x = origin_link_goal_transform.getRotation().getX();
        target_pose.orientation.y = origin_link_goal_transform.getRotation().getY();
        target_pose.orientation.z = origin_link_goal_transform.getRotation().getZ();
        target_pose.orientation.w = origin_link_goal_transform.getRotation().getW();

        waypoints.push_back(actual_pose);
        waypoints.push_back(target_pose);
        moveit::core::RobotStatePtr robot_state_ptr = move_group_->getCurrentState();
        moveit_msgs::RobotState robot_state;
        moveit::core::robotStateToRobotStateMsg(*robot_state_ptr, robot_state);
        moveit_msgs::RobotTrajectory moveit_trj;
        double plan_time = ros::Time::now().toSec();
        if ( move_group_->computeCartesianPath(waypoints,0.01,0.0,moveit_trj) < 0.0 )
        {
            ROS_RED_STREAM("Linear_plan_result: unknown error");
            ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
            return skills_executer_msgs::SkillExecutionResponse::Fail;
        }
        plan_time = ros::Time::now().toSec() - plan_time;
        moveit_plan_.planning_time_ = plan_time;
        moveit_plan_.start_state_   = robot_state;
        moveit_plan_.trajectory_    = moveit_trj;
    }

    moveit::core::MoveItErrorCode move_result = move_group_->execute(moveit_plan_);

    if ( !changeConfig(watch_type_) )
    {
        ROS_RED_STREAM("Problem with configuration manager");
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    ROS_WHITE_STREAM("Move_result: "<<move_result);
    if ( move_result != moveit::core::MoveItErrorCode::SUCCESS )
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

bool SkillsExec::changeConfig(const std::string config_name)
{
    if (use_change_config_bridge_)
    {
        skills_util_msgs::ChangeConfig change_config_srv;
        change_config_srv.request.config_name = config_name;
        change_config_srv.request.robot_name = robot_name_;

        if (!change_config_clnt_.call(change_config_srv))
        {
          ROS_ERROR("Unable to call %s service to set controller %s",change_config_clnt_.getService().c_str(),config_name.c_str());
          return false;
        }

        if (!change_config_srv.response.ok)
        {
          ROS_ERROR("Error on service %s response", change_config_clnt_.getService().c_str());
          return false;
        }
    }
    else
    {
        configuration_msgs::StartConfiguration start_config_srv;
        start_config_srv.request.start_configuration = config_name;
        start_config_srv.request.strictness = 1;

        if (!change_config_clnt_.call(start_config_srv))
        {
          ROS_ERROR("Unable to call %s service to set controller %s",change_config_clnt_.getService().c_str(),config_name.c_str());
          return false;
        }

        if (!start_config_srv.response.ok)
        {
          ROS_ERROR("Error on service %s response", change_config_clnt_.getService().c_str());
          return false;
        }
    }


    ROS_WHITE_STREAM("Controller "<<config_name<<" started.");

//    ros::Duration(0.1).sleep();
    return true;
}

int SkillsExec::reset_ur10e_ft_sensor()
{
    ROS_WHITE_STREAM("In reset_ur10e_ft_sensor");
    ros::ServiceClient reset_force_sensor_clnt = n_.serviceClient<std_srvs::Trigger>("/ur10e_hw/zero_ftsensor");
    if (!reset_force_sensor_clnt.waitForExistence(ros::Duration(2)))
    {
        ROS_ERROR_STREAM(reset_force_sensor_clnt.getService().c_str() << " don't connect.");
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    std_srvs::Trigger zero_srv;
    if ( !reset_force_sensor_clnt.call(zero_srv) )
    {
        ROS_RED_STREAM("Unable to reset force sensor");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    if ( zero_srv.response.success )
        return skills_executer_msgs::SkillExecutionResponse::Success;
    else
        return skills_executer_msgs::SkillExecutionResponse::Fail;
}

int SkillsExec::reset_pybullet_ft_sensor()
{
    ROS_WHITE_STREAM("In reset_pybullet_ft_sensor");
    if (sensored_joint_.empty())
    {
        ROS_INFO_STREAM("Sensored joint name is empty.");
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    ros::ServiceClient reset_force_sensor_clnt = n_.serviceClient<pybullet_simulation::SensorReset>("/pybullet_sensor_reset");
    ROS_WARN("Waiting for %s", reset_force_sensor_clnt.getService().c_str() );
    if (!reset_force_sensor_clnt.waitForExistence(ros::Duration(2)))
    {
        ROS_ERROR_STREAM(reset_force_sensor_clnt.getService().c_str() << " don't connect.");
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    ROS_WARN("Connection ok");

    pybullet_simulation::SensorReset zero_srv;
    zero_srv.request.robot_name = robot_name_;
    zero_srv.request.joint_name = sensored_joint_;
    if ( !reset_force_sensor_clnt.call(zero_srv) )
    {
        ROS_RED_STREAM("Unable to reset force sensor");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    ros::Duration(0.1).sleep();
    if ( !zero_srv.response.result.compare("true") )
        return skills_executer_msgs::SkillExecutionResponse::Success;
    else
        return skills_executer_msgs::SkillExecutionResponse::Fail;
}

int SkillsExec::reset_ati_sensor()
{
    ROS_WHITE_STREAM("In reset_ati_sensor");
    ros::ServiceClient reset_force_sensor_clnt = n_.serviceClient<std_srvs::Trigger>("/battery_cell_utils/reset_ati_sensor");
    ROS_WARN("Waiting for %s", reset_force_sensor_clnt.getService().c_str() );
    if (!reset_force_sensor_clnt.waitForExistence(ros::Duration(2)))
    {
        ROS_ERROR_STREAM(reset_force_sensor_clnt.getService().c_str() << " don't connect.");
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    ROS_WARN("Connection ok");

    std_srvs::Trigger reset_srv;
    if ( !reset_force_sensor_clnt.call(reset_srv) )
    {
        ROS_RED_STREAM("Unable to reset force sensor");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    ros::Duration(0.1).sleep();
    if ( reset_srv.response.success )
        return skills_executer_msgs::SkillExecutionResponse::Success;
    else
        return skills_executer_msgs::SkillExecutionResponse::Fail;
}

int SkillsExec::reset_ft_sensor()
{
    ROS_WHITE_STREAM("In reset_ft_sensor. Sensor type: "<<sensor_type_);
    int result;
    if (sensor_type_ == "pybullet")
        result = reset_pybullet_ft_sensor();
    else if (sensor_type_ == "ur10e")
        result = reset_ur10e_ft_sensor();
    else if (sensor_type_ == "ati_sensor")
        result = reset_ati_sensor();
    else
        ROS_WARN("The sensor type is not in the list");
        result = skills_executer_msgs::SkillExecutionResponse::Success;
    return result;
}

void SkillsExec::maxWrenchCalculation()
{
    if (sensored_joint_.empty())
    {
        ROS_INFO_STREAM("Sensored joint name is empty.");
        return;
    }
    ROS_INFO_STREAM("In maxWrenchcalculation");
    reset_ft_sensor();

    geometry_msgs::WrenchStamped actual_wrench;
    double force_old;
    while ( !end_force_thread_ )
    {
        if ( wrench_sub_->waitForANewData() )
        {
            actual_wrench = wrench_sub_->getData();
        }
        double force = sqrt( pow(actual_wrench.wrench.force.x, 2.0) + pow(actual_wrench.wrench.force.y, 2.0) + pow(actual_wrench.wrench.force.z, 2.0) );
        if ( force > max_force_ )
        {
            max_force_ = force;
            //            ROS_WHITE_STREAM("Actual max force value: %lf", max_force_);
        }
        if ( abs( force - force_old ) > max_force_variation_ )
        {
            contact_ = true;
        }

        force_old = force;
    }
    return;
}

double SkillsExec::tf_distance (const std::string &reference_tf, const std::string &target_frame)
{
    tf::StampedTransform transform;
    double  distance;
    try
    {
        tf_listener_.lookupTransform( target_frame, reference_tf, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
       ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
    }

    tf::Vector3 pose = transform.getOrigin();
    distance = sqrt( pow(pose.getX(),2) + pow(pose.getY(),2) + pow(pose.getZ(),2));
    return distance;
}

void SkillsExec::gripper_feedback()
{
    sensor_msgs::JointState actual_js;

    while ( !end_gripper_feedback_ )
    {
        if ( js_sub_->waitForANewData() )
        {
            actual_js = js_sub_->getData();
        }
        std::vector<std::string>::iterator index = std::find(actual_js.name.begin(), actual_js.name.end(), "right_finger_joint");

        if ( index !=  actual_js.name.end() )
        {
            if ( abs( actual_js.effort.at(index - actual_js.name.begin()) ) < minimum_gripper_force_ )
            {
                setParam(current_action_name_,current_skill_name_,"fail",1);
                setParam(current_action_name_,"fail",1);
                std::string param_name = "/" + current_grasped_object_ + "/attached";
                n_.setParam(param_name,false);
                ROS_YELLOW_STREAM("No object in the gripper. Current position: "<<actual_js.position.at(index - actual_js.name.begin()));
                break;
            }
//            if ( actual_js.position.at(index - actual_js.name.begin()) < parallel_2f_gripper_closed_position_ + gripper_tollerance_ && actual_js.position.at(index - actual_js.name.begin()) > parallel_2f_gripper_closed_position_ - gripper_tollerance_ )
//            {
//                setParam(current_action_name_,current_skill_name_,"fail",1);
//                setParam(current_action_name_,"fail",1);
//                std::string param_name = "/" + current_grasped_object_ + "/attached";
//                n_.setParam(param_name,false);
//                ROS_YELLOW_STREAM("No object in the gripper. Current position: "<<actual_js.position.at(index - actual_js.name.begin()));
//                break;
//            }
        }
    }
    return;
}

int SkillsExec::joint_move_to(const std::string &action_name, const std::string &skill_name)
{
    ROS_WHITE_STREAM("In joint_move_to");
    if ( !changeConfig("trajectory_tracking") )
    {
        ROS_RED_STREAM("Problem with configuration manager");
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    if ( !fjt_ac_->waitForServer(ros::Duration(10)) )
    {
        ROS_RED_STREAM("Timeout FollowJointTrajectory client for robot "<<robot_name_);
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    ros::Duration(0.5).sleep();
    ROS_WHITE_STREAM("Move info: ");
    std::string target_TF;
    double acc, vel, p_time;
    int r_att;
    std::vector<double> relative_position, relative_quaternion;
    geometry_msgs::TransformStamped transform_stamped;
    tf::Vector3 rel_pos;
    tf::Quaternion rel_quat;
    tf::StampedTransform relative_transform;
    Eigen::Affine3d T_relative;

    if (!getParam(action_name, skill_name, "acceleration_scaling", acc))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/acceleration_scaling is not set, defaul value: 0.5" );
        acc = 0.5;
        setParam(action_name, skill_name, "acceleration_scaling", acc);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/acceleration_scaling: "<<acc);
    }
    move_group_->setMaxAccelerationScalingFactor(acc);
    ROS_WHITE_STREAM("  acceleration_scaling: "<<acc);

    if (!getParam(action_name, skill_name, "velocity_scaling", vel))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/velocity_scaling is not set, defaul value: 0.5" );
        vel = 0.5;
        setParam(action_name, skill_name, "velocity_scaling", vel);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/velocity_scaling: "<<vel);
    }
    move_group_->setMaxVelocityScalingFactor(vel);
    ROS_WHITE_STREAM("  velocity_scaling: "<<vel);

    if (!getParam(action_name, skill_name, "planning_time", p_time))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/planning_time is not set, defaul value: 5.0" );
        p_time = 0.5;
        setParam(action_name, skill_name, "planning_time", p_time);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/planning_time: "<<p_time);
    }
    move_group_->setPlanningTime(p_time);
    ROS_WHITE_STREAM("  planning_time: "<<p_time);

    if (!getParam(action_name, skill_name, "replan_attempts", r_att))
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/replan_attempts is not set, defaul value: 10" );
        r_att = 10;
        setParam(action_name, skill_name, "replan_attempts", r_att);
        ROS_WHITE_STREAM("  Set "<<action_name<<"/"<<skill_name<<"/replan_attempts: "<<r_att);
    }
    move_group_->setReplanAttempts(r_att);
    ROS_WHITE_STREAM("  replan_attempts: "<<r_att);

    if (!getParam(action_name, skill_name, "target_frame", target_TF))
    {
        ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/target_TF is not set" );
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    if ( !getParam(action_name, skill_name, "relative_position", relative_position) )
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/relative_position is not set. It is considered [0,0,0]" );
        relative_position.clear();
        relative_position.push_back(0);
        relative_position.push_back(0);
        relative_position.push_back(0);
        setParam(action_name, skill_name, "relative_position", relative_position);
        rel_pos.setX(0);
        rel_pos.setY(0);
        rel_pos.setZ(0);
    }
    else
    {
        if ( relative_position.size() != 3 )
        {
            ROS_YELLOW_STREAM("The relative_position size is not 3");
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_position is considered [0,0,0]" );
            relative_position.clear();
            relative_position.push_back(0);
            relative_position.push_back(0);
            relative_position.push_back(0);
            setParam(action_name, skill_name, "relative_position", relative_position);
            rel_pos.setX(0);
            rel_pos.setY(0);
            rel_pos.setZ(0);
        }
        else
        {
            rel_pos.setX(relative_position.at(0));
            rel_pos.setY(relative_position.at(1));
            rel_pos.setZ(relative_position.at(2));
            ROS_WHITE_STREAM("  relative_position: ["<<relative_position.at(0)<<","<<relative_position.at(1)<<","<<relative_position.at(2)<<"]");
        }
    }
    if ( !getParam(action_name, skill_name, "relative_orientation", relative_quaternion) )
    {
        ROS_YELLOW_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/relative_orientation is not set. It is considered [0,0,0,1]" );
        relative_quaternion.clear();
        relative_quaternion.push_back(0);
        relative_quaternion.push_back(0);
        relative_quaternion.push_back(0);
        relative_quaternion.push_back(1);
        setParam(action_name, skill_name, "relative_orientation", relative_quaternion);
        rel_quat.setX(0);
        rel_quat.setY(0);
        rel_quat.setZ(0);
        rel_quat.setW(1);
    }
    else
    {
        if ( relative_quaternion.size() != 4 )
        {
            ROS_YELLOW_STREAM("The relative_quaternion size is not 4");
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_orientation is considered [0,0,0,1]" );
            relative_quaternion.clear();
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(0);
            relative_quaternion.push_back(1);
            setParam(action_name, skill_name, "relative_orientation", relative_quaternion);
            rel_quat.setX(0);
            rel_quat.setY(0);
            rel_quat.setZ(0);
            rel_quat.setW(1);
        }
        else
        {
            rel_quat.setX(relative_quaternion.at(0));
            rel_quat.setY(relative_quaternion.at(1));
            rel_quat.setZ(relative_quaternion.at(2));
            rel_quat.setW(relative_quaternion.at(3));
            ROS_WHITE_STREAM("  relative_orientation: ["<<relative_quaternion.at(0)<<","<<relative_quaternion.at(1)<<","<<relative_quaternion.at(2)<<","<<relative_quaternion.at(3)<<"]");
        }
    }
    relative_transform.setOrigin(rel_pos);
    relative_transform.setRotation(rel_quat);
    tf::transformTFToEigen(relative_transform, T_relative);

    tf::StampedTransform relative_end_link_transform;
    Eigen::Affine3d T_relative_end_link;

    T_relative_end_link = T_relative * T_gripper_to_end_link_;
    tf::transformEigenToTF(T_relative_end_link, relative_end_link_transform);


    std::string TF_name = target_TF;
    TF_name.append("_goal");
    transform_stamped.transform.translation.x = relative_end_link_transform.getOrigin().getX();
    transform_stamped.transform.translation.y = relative_end_link_transform.getOrigin().getY();
    transform_stamped.transform.translation.z = relative_end_link_transform.getOrigin().getZ();
    transform_stamped.transform.rotation.x = relative_end_link_transform.getRotation().getX();
    transform_stamped.transform.rotation.y = relative_end_link_transform.getRotation().getY();
    transform_stamped.transform.rotation.z = relative_end_link_transform.getRotation().getZ();
    transform_stamped.transform.rotation.w = relative_end_link_transform.getRotation().getW();
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = target_TF;
    transform_stamped.child_frame_id = TF_name;

    ROS_INFO_STREAM("Target tf: "<<target_TF);

    tf_br_.sendTransform(transform_stamped);

    move_group_->clearPoseTargets();

    ik_solver_msgs::GetIk get_ik_msg;
    ik_solver_msgs::Configuration ik_configuration;

    ik_configuration.configuration = move_group_->getCurrentJointValues();

    get_ik_msg.request.tf_name = TF_name;
    get_ik_msg.request.seeds.push_back(ik_configuration);
    get_ik_msg.request.seed_joint_names = move_group_->getJointNames();
    get_ik_msg.request.max_number_of_solutions = 6;
    get_ik_msg.request.stall_iterations = 10;

    int max_iter = 5;
    int iter = 0;

    while ( ros::ok() )
    {
        if (!get_ik_clnt_.call(get_ik_msg))
        {
            ROS_ERROR("Unable to call %s service",get_ik_clnt_.getService().c_str());
            return skills_executer_msgs::SkillExecutionResponse::Fail;
        }
        if ( get_ik_msg.response.solution.configurations.size() != 0 )
        {
            break;
        }
        else
        {
            iter += 1;
            if ( iter == max_iter)
            {
                ROS_RED_STREAM("Goal configuration not found");
                return skills_executer_msgs::SkillExecutionResponse::Fail;
            }
        }
    }

    move_group_->setJointValueTarget(get_ik_msg.response.solution.configurations.at(0).configuration);

    moveit::core::MoveItErrorCode plan_result = move_group_->plan(moveit_plan_);
    ROS_WHITE_STREAM("Plan_result: "<<plan_result);
    if ( plan_result != moveit::core::MoveItErrorCode::SUCCESS )
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        if ( !changeConfig(watch_type_) )
        {
            ROS_RED_STREAM("Problem with configuration manager");
            return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
        }
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    moveit::core::MoveItErrorCode move_result = move_group_->execute(moveit_plan_);

    if ( !changeConfig(watch_type_) )
    {
        ROS_RED_STREAM("Problem with configuration manager");
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    ROS_WHITE_STREAM("Move_result: "<<move_result);
    if ( move_result != moveit::core::MoveItErrorCode::SUCCESS )
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    std::vector<double> target_conf = get_ik_msg.response.solution.configurations.at(0).configuration;
    std::vector<double> current_conf = move_group_->getCurrentJointValues();

    ROS_WHITE_STREAM("Target configurtion: ["<<target_conf.at(0)<<","<<target_conf.at(1)<<","<<target_conf.at(2)<<","
                   <<target_conf.at(3)<<","<<target_conf.at(4)<<","<<target_conf.at(5)<<"]");
    ROS_WHITE_STREAM("Current configurtion: ["<<current_conf.at(0)<<","<<current_conf.at(1)<<","<<current_conf.at(2)<<","
                   <<current_conf.at(3)<<","<<current_conf.at(4)<<","<<current_conf.at(5)<<"]");

    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::releaseEndEffector(const std::string &action_name, const std::string &skill_name)
{
    reference_end_effector_frame_ = end_link_frame_;
    attached_link_name_.clear();
    end_effector_touch_links_.clear();

    tf::StampedTransform gripper_link_transform;
    try
    {
        tf_listener_.lookupTransform( reference_end_effector_frame_, end_link_frame_, ros::Time(0), gripper_link_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::transformTFToEigen( gripper_link_transform, T_gripper_to_end_link_);

    //    Command to release the end effector

}

int SkillsExec::attachEndEffector(const std::string &action_name, const std::string &skill_name)
{
    if (!getParam(action_name, skill_name, "reference_end_effector_frame", reference_end_effector_frame_))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/reference_end_effector_frame is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "attached_link_name",   attached_link_name_))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/attached_link_name is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "end_effector_touch_links", end_effector_touch_links_))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/end_effector_touch_links is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    tf::StampedTransform gripper_link_transform;
    try
    {
        tf_listener_.lookupTransform( reference_end_effector_frame_, end_link_frame_, ros::Time(0), gripper_link_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::transformTFToEigen( gripper_link_transform, T_gripper_to_end_link_);

//    Command to attach the end effector
}

} // end namespace skills_executer
