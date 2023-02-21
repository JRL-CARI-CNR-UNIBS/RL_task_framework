#include <skills_executer/skills_exec.h>

namespace skills_executer
{

SkillsExec::SkillsExec(const ros::NodeHandle & n) : n_(n)
{
    twist_pub_        = n_.advertise<geometry_msgs::TwistStamped>("/target_cart_twist",1);

    wrench_sub_ = std::make_shared<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>>(n_, "/gripper/wrench", 10);
    js_sub_= std::make_shared<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>>(n_, "/joint_states", 10);

    skill_exec_srv_ = n_.advertiseService("/skills_exec/execute_skill", &SkillsExec::skillsExecution, this);

    start_config_clnt_ = n_.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
    ROS_YELLOW_STREAM("Waiting for "<<start_config_clnt_.getService());
    start_config_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");

//    skill_arbit_clnt_ = n_.serviceClient<skills_arbitrator_msgs::SkillArbitration>("/skills_arbit/evaluate_skill");
//    ROS_YELLOW_STREAM("Waiting for "<<skill_arbit_clnt_.getService());
//    skill_arbit_clnt_.waitForExistence();
//    ROS_YELLOW_STREAM("Connection ok");

    touch_action_         = std::make_shared<actionlib::SimpleActionClient<simple_touch_controller_msgs::SimpleTouchAction>>("simple_touch", true);
    relative_move_action_ = std::make_shared<actionlib::SimpleActionClient<relative_cartesian_controller_msgs::RelativeMoveAction>>("relative_move", true);
    move_group_           = std::make_shared<moveit::planning_interface::MoveGroupInterface>(robot_name_);
    screw_accuracy_ = 0.1;

    tf::StampedTransform gripper_link_transform;
    try
    {
        tf_listener_.lookupTransform( gripper_frame_, end_link_frame_, ros::Time(0), gripper_link_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::transformTFToEigen( gripper_link_transform, T_gripper_link_);

    //    fjt part
    fjt_ac_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+robot_name_+"/follow_joint_trajectory",true));
    for ( std::size_t i = 0; i < move_group_->getJointNames().size(); i++)
    {
        trajectory_joint_tollerance_.push_back(0.001);
    }
    //    end

}

bool SkillsExec::skillsExecution(skills_executer_msgs::SkillExecution::Request  &req,
                                 skills_executer_msgs::SkillExecution::Response &res)
{
    ROS_MAGENTA_STREAM("Skill requested: "<<req.action_name<<"/"<<req.skill_name);
    std::string skill_type;
    std::string action_type;
    std::string object_name;
    std::string location_name;
    double initial_distance, final_distance, traveled_distance, total_traveled_distance;

    // Is it right to put arbitrator here?
//    if ( !req.skill_name.compare("end") )
//    {
//        skills_arbitrator_msgs::SkillArbitration skill_arbit_srv;
//        skill_arbit_srv.request.action_name = req.action_name;
//        if ( !skill_arbit_clnt_.call(skill_arbit_srv) )
//        {
//            ROS_ERROR("Fail to call service: %s", skill_arbit_clnt_.getService().c_str());
//            ROS_ERROR("action_name: %s", skill_arbit_srv.request.action_name.c_str());
//            res.result = skills_executer_msgs::SkillExecutionResponse::Error;
//            return true;
//        }
//        res.result = skills_executer_msgs::SkillExecutionResponse::Success;
//        return true;
//    }
    //
    if (!getParam(req.action_name, "action_type",   action_type))
    {
        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/action_type, skill execution finish");
        res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
        return true;
    }
    ROS_WHITE_STREAM("Action type: "<<action_type);

    if (!getParam(req.action_name, req.skill_name, "skill_type",   skill_type))
    {
        ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/"<<req.skill_name<<"/skill_type, skill execution finish");
        res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
        return true;
    }
    ROS_WHITE_STREAM("Skill type: "<<skill_type);

    setParam(req.action_name,"executed",1);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/executed: "<<1);

    if ( !action_type.compare("pick") )
    {
        if (!getParam(req.action_name, "object_name", object_name))
        {
            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/object_name, skill execution finish");
            res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
            return true;
        }
        ROS_WHITE_STREAM("Pick object_name: "<<object_name);
        initial_distance = tf_distance(gripper_frame_, object_name);
    }
    else if( !action_type.compare("place") )
    {
        if (!getParam(req.action_name, "location_name", location_name))
        {
            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/location_name, skill execution finish");
            res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
            return true;
        }
        ROS_WHITE_STREAM("Place location_name: "<<location_name);
        if (!getParam(req.action_name, "object_name", object_name))
        {
            ROS_YELLOW_STREAM("No param /"<<req.action_name<<"/object_name, skill execution finish");
            res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
            return true;
        }
        ROS_WHITE_STREAM("Place object_name: "<<object_name);
//        initial_distance = tf_distance(gripper_frame_, location_name);
        initial_distance = tf_distance(object_name, location_name);
    }
    else
    {
        initial_distance = 0;
    }
    ROS_WHITE_STREAM("Initial distance: "<<initial_distance);

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
        res.result = cartPos(req.action_name, req.skill_name);
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
    else if ( !skill_type.compare(ur_load_program_) )
    {
        res.result = urLoadProgram(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(move_to_type_) )
    {
        res.result = move_to(req.action_name, req.skill_name);
//        res.result = follow_joint_trj(req.action_name, req.skill_name, false);
    }
    else if ( !skill_type.compare(linear_move_type_) )
    {
//        res.result = move_to(req.action_name, req.skill_name);
        res.result = follow_joint_trj(req.action_name, req.skill_name, true);
    }
    else
    {
        ROS_YELLOW_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<" result: NoSkillType");
        res.result = skills_executer_msgs::SkillExecutionResponse::NoSkillType;
        ROS_YELLOW_STREAM("Return true");
        return true;
    }

    int int_result = res.result;
    ROS_MAGENTA_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<" result: "<<int_result);

    setParam(req.action_name,req.skill_name,"executed",1);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/executed: "<<1);

    if ( res.result != skills_executer_msgs::SkillExecutionResponse::Success )
    {
        setParam(req.action_name, "fail", 1);
        ROS_YELLOW_STREAM("Set /"<<req.action_name<<"/fail: "<<1);
    }

    double duration = ros::Time::now().toSec() - initial_time.toSec();

    end_force_thread_ = true;

    wrench_thread.join();

    end_force_thread_ = false;

    setParam(req.action_name,req.skill_name,"duration",duration);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/duration: "<<duration);
    setParam(req.action_name,req.skill_name,"max_force",max_force_);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/max_force: "<<max_force_);
    setParam(req.action_name,req.skill_name,"contact",contact_);
    ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/contact: "<<contact_);

    if ( !action_type.compare("pick") )
    {
        final_distance = tf_distance(gripper_frame_, object_name);
        ROS_WHITE_STREAM("Final distance: "<<final_distance);
        traveled_distance = initial_distance - final_distance;
        setParam(req.action_name,req.skill_name,"traveled_distance",traveled_distance);
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/"<<req.skill_name<<"/traveled_distance: "<<traveled_distance);
        if (!getParam(req.action_name, "traveled_distance", total_traveled_distance))
        {
            ROS_YELLOW_STREAM("No param: /"<<req.action_name<<"/traveled_distance");
            total_traveled_distance = 0.0;
            setParam(req.action_name,"traveled_distance",total_traveled_distance);
            ROS_YELLOW_STREAM("/"<<req.action_name<<"/traveled_distance: "<<total_traveled_distance);
        }
        total_traveled_distance = total_traveled_distance + traveled_distance;
        setParam(req.action_name,"traveled_distance",total_traveled_distance);
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/traveled_distance: "<<total_traveled_distance);
    }
    else if( !action_type.compare("place") )
    {
//        final_distance = tf_distance(gripper_frame_, location_name);
        final_distance = tf_distance(object_name, location_name);
        double place_tollerance;
        ROS_WHITE_STREAM("Final distance: "<<final_distance);
        traveled_distance = initial_distance - final_distance;
        setParam(req.action_name,req.skill_name,"traveled_distance",traveled_distance);
        ROS_WHITE_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<"/traveled_distance: "<<traveled_distance);
        if (!getParam(req.action_name, "traveled_distance", total_traveled_distance))
        {
            ROS_YELLOW_STREAM("No param: /"<<req.action_name<<"/traveled_distance, it is considered equal to 0");
            total_traveled_distance = 0.0;
            setParam(req.action_name,"traveled_distance",total_traveled_distance);
            ROS_YELLOW_STREAM("/"<<req.action_name<<"/traveled_distance: "<<total_traveled_distance);
       }
        total_traveled_distance = total_traveled_distance + traveled_distance;
        setParam(req.action_name,"traveled_distance",total_traveled_distance);
        ROS_WHITE_STREAM("Set /"<<req.action_name<<"/traveled_distance: "<<total_traveled_distance);
        if (!getParam(req.action_name, "tollerance", place_tollerance))
        {
            ROS_YELLOW_STREAM("No param: /"<<req.action_name<<"/place_tollerance, default: 0.01");
            place_tollerance = 0.01;
        }
        ROS_WHITE_STREAM("Place_tollerance: "<<place_tollerance);
        if ( final_distance < place_tollerance )
        {
            ROS_WHITE_STREAM("Object in tollerance. Fail set to 0");
            setParam(req.action_name,"fail",0);
        }
        else
        {
            ROS_YELLOW_STREAM("Object not in tollerance. Fail set to 1");
            setParam(req.action_name,"fail",1);
        }
    }

    ROS_MAGENTA_STREAM("/"<<req.action_name<<"/"<<req.skill_name<<" skill execution finish, return true");
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
    parallel_gripper_move_clnt_ = n_.serviceClient<parallel_2f_gripper::MoveGripper>("/move_parallel_gripper");
    ROS_YELLOW_STREAM("Waiting for "<<parallel_gripper_move_clnt_.getService());
    parallel_gripper_move_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");

    setParam(action_name,skill_name,"fail",0);
    ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/fail: 0");

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
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<": torque parameter is missing. Return NoParam");
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
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<": velocity parameter is missing. Return NoParam");
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
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<": position parameter is missing. Return NoParam");
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
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<": some parameters are missing");
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
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
    else if (control_mode == "torque" & torque > 0)
    {
        closure = true;
    }
    else if (control_mode == "velocity" & velocity > 0)
    {
        closure = true;
    }
    else
    {
        closure = false;
    }

    std::string param_name = "/" + current_grasped_object_ + "/attached";
    if ( closure & !thread_esistence_ )
    {
        ROS_ERROR("In if");
        end_gripper_feedback_ = false;
        current_action_name_ = action_name;
        current_skill_name_  = skill_name;
        gripper_thread_ = std::make_shared<std::thread>([this]{gripper_feedback();});
        thread_esistence_ = true;
        n_.setParam(param_name,true);
        param_name.append("_robot");
        n_.setParam(param_name,robot_name_);
    }
    else
    {
        ROS_ERROR("In else");
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

    if (!parallel_gripper_move_clnt_.call(move_gripper_srv))
    {
      ROS_ERROR("Unable to call %s service ",parallel_gripper_move_clnt_.getService().c_str());
      return false;
    }

    ROS_RED_STREAM("Parallel_gripper_move return :"<<move_gripper_srv.response.result);
    if ( !move_gripper_srv.response.result.compare("true") )
    {
        ROS_MAGENTA_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    else
    {
        ROS_MAGENTA_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ros::Duration(1.0).sleep();

    ROS_MAGENTA_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::robotiqGripperMove(const std::string &action_name, const std::string &skill_name)
{
    ros::ServiceClient gripper_clnt = n_.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");
    ROS_YELLOW_STREAM("Waiting for "<<gripper_clnt.getService() );
    gripper_clnt.waitForExistence();
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

//    if ( !changeConfig(watch_config_) )
//    {
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
//    }

    if ( contact_ )
    {
        ROS_YELLOW_STREAM("Contact! /"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::cartPos(const std::string &action_name, const std::string &skill_name)
{
    double rotZdeg, rotYdeg, rotXdeg, traXmm, traYmm, traZmm, target_angular_velocity, target_linear_velocity, vel;
    std::string skill_type;
    geometry_msgs::PoseStamped relative_pose;
    tf2::Quaternion quat;
    std::vector<double> position, orientation;

    if (!getParam(action_name, skill_name, "skill_type", skill_type))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/skill_type is not set" );
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if( getParam(action_name, skill_name, "rotZdeg", rotZdeg) )
    {
        double angle = rotZdeg*pi_/180;
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
        double angle = rotYdeg*pi_/180;
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
        double angle = rotXdeg*pi_/180;
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
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/position is not set" );
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( !position.size() == 3 )
            {
                ROS_YELLOW_STREAM("The position size is not 3");
                ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            relative_pose.pose.position.x = position.at(0);
            relative_pose.pose.position.y = position.at(1);
            relative_pose.pose.position.z = position.at(2);
            ROS_WHITE_STREAM("Read position: ["<<relative_pose.pose.position.x<<","<<relative_pose.pose.position.y<<","<<relative_pose.pose.position.z<<"]");
        }
        if ( !getParam(action_name, skill_name, "quaternion", orientation) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/quaternion is not set" );
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( !orientation.size() == 4 )
            {
                ROS_YELLOW_STREAM("The quaternion size is not 4");
                ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            relative_pose.pose.orientation.x = orientation.at(0);
            relative_pose.pose.orientation.y = orientation.at(1);
            relative_pose.pose.orientation.z = orientation.at(2);
            relative_pose.pose.orientation.w = orientation.at(3);
            ROS_WHITE_STREAM("Read quaternion: ["<<relative_pose.pose.orientation.x<<","<<relative_pose.pose.orientation.y<<","<<relative_pose.pose.orientation.z<<","<<relative_pose.pose.orientation.w<<"]");
        }
    }
    if ( !getParam(action_name, skill_name, "frame", relative_pose.header.frame_id) )
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/frame is not set" );
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
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
            target_angular_velocity = 30*pi_/180;
        }
        else
        {
            target_angular_velocity = vel*pi_/180;
            ROS_WHITE_STREAM("Read angular_velocity_deg_s: "<<vel);
            ROS_WHITE_STREAM("Angular_velocity_rad_s: "<<target_angular_velocity);
        }
    }
    else
    {
        ROS_WHITE_STREAM("Read angular_velocity_rad_s: "<<target_angular_velocity);
    }

    if ( !changeConfig(skill_type) )
    {
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return ProblemConfManager: "<<skills_executer_msgs::SkillExecutionResponse::ProblemConfManager);
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    relative_cartesian_controller_msgs::RelativeMoveGoal rel_move_goal;
    rel_move_goal.target_angular_velocity = target_angular_velocity;
    rel_move_goal.target_linear_velocity = target_linear_velocity;
    rel_move_goal.relative_pose = relative_pose;

    ROS_WHITE_STREAM("Goal:");
    ROS_WHITE_STREAM("Frame: "<<rel_move_goal.relative_pose.header.frame_id);
    ROS_WHITE_STREAM("Position: ["<<rel_move_goal.relative_pose.pose.position.x<<","<<rel_move_goal.relative_pose.pose.position.y<<","<<rel_move_goal.relative_pose.pose.position.z<<"]");
    ROS_WHITE_STREAM("Quaternion: ["<<rel_move_goal.relative_pose.pose.orientation.x<<","<<rel_move_goal.relative_pose.pose.orientation.y<<","<<rel_move_goal.relative_pose.pose.orientation.z<<","<<rel_move_goal.relative_pose.pose.orientation.w<<"]");
    ROS_WHITE_STREAM("Velocity: lin "<<rel_move_goal.target_linear_velocity<<", rot "<<rel_move_goal.target_angular_velocity);

    relative_move_action_->waitForServer();
    relative_move_action_->sendGoalAndWait(rel_move_goal);

//    if ( !changeConfig(watch_config_) )
//    {
//        ROS_BOLDMAGENTA_STREAM("/"<<action_name<<"/"<<skill_name<<" return ProblemConfManager: "<<skills_executer_msgs::SkillExecutionResponse::ProblemConfManager);
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
//    }

    if ( contact_ )
    {
        ROS_YELLOW_STREAM("Contact! /"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    if ( relative_move_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_BOLDMAGENTA_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    else
    {
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
}

int SkillsExec::simpleTouch(const std::string &action_name, const std::string &skill_name)
{
    std::string         skill_type;
    std::string         goal_twist_frame;
    std::vector<double> goal_twist;

    double target_force;
    bool   relative_target;
    double release;
    int    release_condition;

    if (!getParam(action_name, skill_name, "skill_type", skill_type) )
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/skill_type is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> skill_type: "<<skill_type);
    if (!getParam(action_name, skill_name, "goal_twist_frame", goal_twist_frame))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/goal_twist_frame is not set");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> goal_twist_frame: "<<goal_twist_frame);
    if (!getParam(action_name, skill_name, "goal_twist", goal_twist))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/goal_twist is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
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
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<"-> goal_twist has wrong size");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    if (!getParam(action_name, skill_name, "target_force", target_force))
    {
        if (!getParam(action_name, skill_name, "target_wrench", target_force))
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/target_wrench or target_force are not set" );
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
    }
    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> target_wrench: "<<target_force);

    if (!getParam(action_name, skill_name, "release", release))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/release is not set" );
    }
    else
    {
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> release: "<<release);
    }

    if (!getParam(action_name, skill_name, "release_condition", release_condition))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/release_condition is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    else
    {
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> release_condition: "<<release_condition);
    }

    if (!getParam(action_name, skill_name, "relative_target", relative_target))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/relative_target is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (relative_target)
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> relative_target: true");
    else
        ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<"-> relative_target: false");

    ROS_WHITE_STREAM("Change configuration: "<<skill_type );

    if ( !changeConfig(skill_type) )
    {
        ROS_YELLOW_STREAM("Problem with configuration manager" );
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

//    if ( !changeConfig(watch_config_) )
//    {
//        ROS_YELLOW_STREAM("Problem with configuration manager" );
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
//    }

    if ( touch_action_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WHITE_STREAM("Simple Touch success");
        return skills_executer_msgs::SkillExecutionResponse::Success;
    }
    else
    {
        ROS_YELLOW_STREAM("Simple touch failed" );
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

}

int SkillsExec::move_to(const std::string &action_name, const std::string &skill_name)
{
    ROS_GREEN_STREAM("Move_to info: ");
    std::string target_TF;
    double acc, vel, p_time, goal_t, goal_j_t, start_t, goal_duration_m;
    bool exec_duration_m;
    int r_att;

    tf::StampedTransform origin_goal_transform, origin_link_goal_transform;


    if (!getParam(action_name, skill_name, "target_frame", target_TF))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/target_TF is not set" );
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
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
    ROS_GREEN_STREAM("  target_frame: "<<target_TF);

    Eigen::Affine3d T_origin_gripper;

    tf::transformTFToEigen( origin_goal_transform, T_origin_gripper);

    Eigen::Affine3d T_origin_link_goal = T_origin_gripper * T_gripper_link_;

    tf::transformEigenToTF(T_origin_link_goal,origin_link_goal_transform);

    geometry_msgs::Pose target_pose;

    target_pose.position.x = origin_link_goal_transform.getOrigin().getX();
    target_pose.position.y = origin_link_goal_transform.getOrigin().getY();
    target_pose.position.z = origin_link_goal_transform.getOrigin().getZ();
    target_pose.orientation.x = origin_link_goal_transform.getRotation().getX();
    target_pose.orientation.y = origin_link_goal_transform.getRotation().getY();
    target_pose.orientation.z = origin_link_goal_transform.getRotation().getZ();
    target_pose.orientation.w = origin_link_goal_transform.getRotation().getW();

    move_group_->setPoseTarget(target_pose);

    if (!getParam(action_name, skill_name, "acceleration_scaling", acc))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/acceleration_scaling is not set, defaul value: 0.5" );
        acc = 0.5;
        setParam(action_name, skill_name, "acceleration_scaling", acc);
        ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/acceleration_scaling: "<<acc);
    }  
    move_group_->setMaxAccelerationScalingFactor(acc);
    ROS_GREEN_STREAM("  acceleration_scaling: "<<acc);
   
    if (!getParam(action_name, skill_name, "velocity_scaling", vel))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/velocity_scaling is not set, defaul value: 0.5" );
        vel = 0.5;
        setParam(action_name, skill_name, "velocity_scaling", vel);
        ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/velocity_scaling: "<<vel);
    }
    move_group_->setMaxVelocityScalingFactor(vel);
    ROS_GREEN_STREAM("  velocity_scaling: "<<vel);

    if (!getParam(action_name, skill_name, "planning_time", p_time))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/planning_time is not set, defaul value: 5.0" );
        p_time = 0.5;
    }  
    move_group_->setPlanningTime(p_time);
    ROS_GREEN_STREAM("  planning_time: "<<p_time);

    if (!getParam(action_name, skill_name, "replan_attempts", r_att))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/replan_attempts is not set, defaul value: 10" );
        r_att = 10;
        setParam(action_name, skill_name, "replan_attempts", r_att);
        ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/replan_attempts: "<<r_att);
    }
    move_group_->setReplanAttempts(r_att);
    ROS_GREEN_STREAM("  replan_attempts: "<<r_att);

    moveit::core::MoveItErrorCode plan_result = move_group_->plan(moveit_plan_);

    if ( !changeConfig("trajectory_tracking") )
    {
        ROS_YELLOW_STREAM("Problem with configuration manager");
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    ROS_YELLOW_STREAM("Plan_result: "<<plan_result);
    if ( plan_result != moveit::core::MoveItErrorCode::SUCCESS )
    {
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
//        changeConfig(watch_config_);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    moveit::core::MoveItErrorCode move_result = move_group_->execute(moveit_plan_);

//    provo a confrontare la posizione finale della trj e quella reale raggiunta.

    std::vector<double> final_config = moveit_plan_.trajectory_.joint_trajectory.points.back().positions;
    std::vector<double> actual_config =  move_group_->getCurrentJointValues();

    ROS_RED_STREAM("Final config: ["<<final_config[0]<<","<<final_config[1]<<","<<final_config[2]<<","<<final_config[3]<<","<<final_config[4]<<","<<final_config[5]<<"]");
    ROS_RED_STREAM("Actual config: ["<<actual_config[0]<<","<<actual_config[1]<<","<<actual_config[2]<<","<<actual_config[3]<<","<<actual_config[4]<<","<<actual_config[5]<<"]");
    ROS_RED_STREAM("Diff: ["<<final_config[0] - actual_config[0]<<","<<
                              final_config[1] - actual_config[1]<<","<<
                              final_config[2] - actual_config[2]<<","<<
                              final_config[3] - actual_config[3]<<","<<
                              final_config[4] - actual_config[4]<<","<<
                              final_config[5] - actual_config[5]<<"]");
//    if ( !changeConfig(watch_config_) )
//    {
//        ROS_YELLOW_STREAM("Problem with configuration manager");
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
//    }

    ROS_YELLOW_STREAM("Move_result: "<<move_result);
    if ( move_result != moveit::core::MoveItErrorCode::SUCCESS )
    {
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

bool SkillsExec::changeConfig(std::string config_name)
{
    configuration_msgs::StartConfiguration start_config_srv;
    start_config_srv.request.start_configuration = config_name;
    start_config_srv.request.strictness = 1;

    if (!start_config_clnt_.call(start_config_srv))
    {
      ROS_ERROR("Unable to call %s service to set controller %s",start_config_clnt_.getService().c_str(),config_name.c_str());
      return false;
    }

    if (!start_config_srv.response.ok)
    {
      ROS_ERROR("Error on service %s response", start_config_clnt_.getService().c_str());
      return false;
    }

    ROS_MAGENTA_STREAM("Controller "<<config_name<<" started.");

//    ros::Duration(0.1).sleep();
    return true;
}

int SkillsExec::reset_ur10e_ft_sensor()
{
    ros::ServiceClient reset_force_sensor_clnt = n_.serviceClient<std_srvs::Trigger>("/ur10e_hw/zero_ftsensor");
    reset_force_sensor_clnt.waitForExistence();
    std_srvs::Trigger zero_srv;
    if ( !reset_force_sensor_clnt.call(zero_srv) )
    {
        ROS_ERROR("Unable to reset force sensor");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    if ( zero_srv.response.success )
        return skills_executer_msgs::SkillExecutionResponse::Success;
    else
        return skills_executer_msgs::SkillExecutionResponse::Fail;
}

void SkillsExec::maxWrenchCalculation()
{
//    geometry_msgs::WrenchStamped actual_wrench;
//    double force_old;
//    while ( !end_force_thread_ )
//    {
//        if ( wrench_sub_->waitForANewData() )
//        {
//            actual_wrench = wrench_sub_->getData();
//        }
//        double force = sqrt( pow(actual_wrench.wrench.force.x, 2.0) + pow(actual_wrench.wrench.force.y, 2.0) + pow(actual_wrench.wrench.force.z, 2.0) );
//        if ( force > max_force_ )
//        {
//            max_force_ = force;
//            //            ROS_MAGENTA_STREAM("Actual max force value: %lf", max_force_);
//        }
//        if ( abs( force - force_old ) > max_force_variation_ )
//        {
//            contact_ = true;
//        }

//        force_old = force;
//    }
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
            if ( abs(actual_js.position.at(index - actual_js.name.begin())) < closed_gripper_position_ + gripper_tollerance_ && abs(actual_js.position.at(index - actual_js.name.begin())) > closed_gripper_position_ - gripper_tollerance_ )
            {
                setParam(current_action_name_,current_skill_name_,"fail",1);
                setParam(current_action_name_,"fail",1);
                std::string param_name = "/" + current_grasped_object_ + "/attached";
                n_.setParam(param_name,false);
                ROS_YELLOW_STREAM("No object in the gripper");
                break;
            }
        }
    }
    return;
}

int SkillsExec::follow_joint_trj(const std::string &action_name, const std::string &skill_name, bool linear_trj)
{
    std::string target_TF, move_reference_frame;
    double acc, vel, p_time;
    int r_att;
    tf::StampedTransform origin_goal_transform, origin_link_goal_transform;
    tf::StampedTransform movement_transform, link_to_reference_transform, reference_to_link_transform, link_to_reference_rotation, reference_to_link_rotation, origin_to_link_transform;
    std::vector<geometry_msgs::Pose> waypoints;
    std::vector<double> position, orientation;
    tf::Vector3 pos;
    tf::Quaternion quat;

    if (!getParam(action_name, skill_name, "acceleration_scaling", acc))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/acceleration_scaling is not set, defaul value: 0.5" );
        acc = 0.5;
        setParam(action_name, skill_name, "acceleration_scaling", acc);
        ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/acceleration_scaling: "<<acc);
    }
    move_group_->setMaxAccelerationScalingFactor(acc);

    if (!getParam(action_name, skill_name, "velocity_scaling", vel))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/velocity_scaling is not set, defaul value: 0.5" );
        vel = 0.5;
        setParam(action_name, skill_name, "velocity_scaling", vel);
        ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/velocity_scaling: "<<vel);
    }
    move_group_->setMaxVelocityScalingFactor(vel);

    if (!getParam(action_name, skill_name, "planning_time", p_time))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/planning_time is not set, defaul value: 5.0" );
        p_time = 0.5;
        setParam(action_name, skill_name, "planning_time", p_time);
        ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/planning_time: "<<p_time);
    }
    move_group_->setPlanningTime(p_time);

    if (!getParam(action_name, skill_name, "replan_attempts", r_att))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/replan_attempts is not set, defaul value: 10" );
        r_att = 10;
        setParam(action_name, skill_name, "replan_attempts", r_att);
        ROS_YELLOW_STREAM("Set "<<action_name<<"/"<<skill_name<<"/replan_attempts: "<<r_att);
    }
    move_group_->setReplanAttempts(r_att);

    control_msgs::FollowJointTrajectoryGoal goal;

    if ( linear_trj )
    {
        geometry_msgs::Pose actual_pose = move_group_->getCurrentPose().pose;
        geometry_msgs::Pose final_pose;

        if ( !getParam(action_name, skill_name, "position", position) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/position is not set" );
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( !position.size() == 3 )
            {
                ROS_YELLOW_STREAM("The position size is not 3");
                ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            pos.setX(position.at(0));
            pos.setY(position.at(1));
            pos.setZ(position.at(2));
            ROS_WHITE_STREAM("Read position: ["<<pos.getX()<<","<<pos.getY()<<","<<pos.getZ()<<"]");
        }
        if ( !getParam(action_name, skill_name, "quaternion", orientation) )
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/quaternion is not set" );
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( !orientation.size() == 4 )
            {
                ROS_YELLOW_STREAM("The quaternion size is not 4");
                ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
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
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/frame is not set" );
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
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

        final_pose.position.x = origin_link_goal_transform.getOrigin().getX();
        final_pose.position.y = origin_link_goal_transform.getOrigin().getY();
        final_pose.position.z = origin_link_goal_transform.getOrigin().getZ();
        final_pose.orientation.x = origin_link_goal_transform.getRotation().getX();
        final_pose.orientation.y = origin_link_goal_transform.getRotation().getY();
        final_pose.orientation.z = origin_link_goal_transform.getRotation().getZ();
        final_pose.orientation.w = origin_link_goal_transform.getRotation().getW();

        waypoints.push_back(actual_pose);
        waypoints.push_back(final_pose);

        moveit_msgs::RobotTrajectory moveit_trj;
        if ( move_group_->computeCartesianPath(waypoints,0.01,0.0,moveit_trj) < 0.0 )
        {
            ROS_YELLOW_STREAM("Linear_plan_result: unknown error");
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
            return skills_executer_msgs::SkillExecutionResponse::Fail;
        }
        goal.trajectory = moveit_trj.joint_trajectory;
    }
    else
    {
        if (!getParam(action_name, skill_name, "target_frame", target_TF))
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/target_TF is not set" );
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return NoParam: "<<skills_executer_msgs::SkillExecutionResponse::NoParam);
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        ROS_WHITE_STREAM("target_frame :"<<target_TF);

        try
        {
            tf_listener_.lookupTransform( "world", target_TF, ros::Time(0), origin_goal_transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        Eigen::Affine3d T_origin_gripper;

        tf::transformTFToEigen( origin_goal_transform, T_origin_gripper);

        Eigen::Affine3d T_origin_link_goal = T_origin_gripper * T_gripper_link_;

        tf::transformEigenToTF(T_origin_link_goal,origin_link_goal_transform);

        geometry_msgs::Pose target_pose;

        target_pose.position.x = origin_link_goal_transform.getOrigin().getX();
        target_pose.position.y = origin_link_goal_transform.getOrigin().getY();
        target_pose.position.z = origin_link_goal_transform.getOrigin().getZ();
        target_pose.orientation.x = origin_link_goal_transform.getRotation().getX();
        target_pose.orientation.y = origin_link_goal_transform.getRotation().getY();
        target_pose.orientation.z = origin_link_goal_transform.getRotation().getZ();
        target_pose.orientation.w = origin_link_goal_transform.getRotation().getW();

        move_group_->setPoseTarget(target_pose);

        moveit::core::MoveItErrorCode plan_result = move_group_->plan(moveit_plan_);

        if ( plan_result != moveit::core::MoveItErrorCode::SUCCESS )
        {
            ROS_YELLOW_STREAM("Result: " << plan_result);
//            switch (plan_result.val) {
//            case moveit::core::MoveItErrorCode::FAILURE :
//                ROS_YELLOW_STREAM("Plan_result: FAILURE");
//                break;
//            case moveit::core::MoveItErrorCode::PLANNING_FAILED :
//                ROS_YELLOW_STREAM("Plan_result: PLANNING_FAILED");
//                break;
//            case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN :
//                ROS_YELLOW_STREAM("Plan_result: INVALID_MOTION_PLAN");
//                break;
//            case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE :
//                ROS_YELLOW_STREAM("Plan_result: MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE");
//                break;
//            case moveit::core::MoveItErrorCode::CONTROL_FAILED :
//                ROS_YELLOW_STREAM("Plan_result: CONTROL_FAILED");
//                break;
//            case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA :
//                ROS_YELLOW_STREAM("Plan_result: UNABLE_TO_AQUIRE_SENSOR_DATA");
//                break;
//            case moveit::core::MoveItErrorCode::TIMED_OUT :
//                ROS_YELLOW_STREAM("Plan_result: TIMED_OUT");
//                break;
//            case moveit::core::MoveItErrorCode::PREEMPTED :
//                ROS_YELLOW_STREAM("Plan_result: PREEMPTED");
//                break;
//            case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION :
//                ROS_YELLOW_STREAM("Plan_result: START_STATE_IN_COLLISION");
//                break;
//            case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS :
//                ROS_YELLOW_STREAM("Plan_result: FAILURE");
//                break;
//            case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION :
//                ROS_YELLOW_STREAM("Plan_result: GOAL_IN_COLLISION");
//                break;
//            case moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS :
//                ROS_YELLOW_STREAM("Plan_result: GOAL_VIOLATES_PATH_CONSTRAINTS");
//                break;
//            case moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED :
//                ROS_YELLOW_STREAM("Plan_result: GOAL_CONSTRAINTS_VIOLATED");
//                break;
//            case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME :
//                ROS_YELLOW_STREAM("Plan_result: INVALID_GROUP_NAME");
//                break;
//            case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS :
//                ROS_YELLOW_STREAM("Plan_result: INVALID_GOAL_CONSTRAINTS");
//                break;
//            case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE :
//                ROS_YELLOW_STREAM("Plan_result: INVALID_ROBOT_STATE");
//                break;
//            case moveit::core::MoveItErrorCode::INVALID_LINK_NAME :
//                ROS_YELLOW_STREAM("Plan_result: INVALID_LINK_NAME");
//                break;
//            case moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME :
//                ROS_YELLOW_STREAM("Plan_result: INVALID_OBJECT_NAME");
//                break;
//            case moveit::core::MoveItErrorCode::FRAME_TRANSFORM_FAILURE :
//                ROS_YELLOW_STREAM("Plan_result: FRAME_TRANSFORM_FAILURE");
//                break;
//            case moveit::core::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE :
//                ROS_YELLOW_STREAM("Plan_result: COLLISION_CHECKING_UNAVAILABLE");
//                break;
//            case moveit::core::MoveItErrorCode::ROBOT_STATE_STALE :
//                ROS_YELLOW_STREAM("Plan_result: ROBOT_STATE_STALE");
//                break;
//            case moveit::core::MoveItErrorCode::SENSOR_INFO_STALE :
//                ROS_YELLOW_STREAM("Plan_result: SENSOR_INFO_STALE");
//                break;
//            case moveit::core::MoveItErrorCode::COMMUNICATION_FAILURE :
//                ROS_YELLOW_STREAM("Plan_result: COMMUNICATION_FAILURE");
//                break;
//            case moveit::core::MoveItErrorCode::NO_IK_SOLUTION :
//                ROS_YELLOW_STREAM("Plan_result: NO_IK_SOLUTION");
//                break;
//            default:
//                ROS_YELLOW_STREAM("Plan_result: unknown error");
//                break;
//            }
            ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
            return skills_executer_msgs::SkillExecutionResponse::Fail;
        }

        goal.trajectory = moveit_plan_.trajectory_.joint_trajectory;
    }

    if ( !changeConfig("trajectory_tracking") )
    {
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    if ( !fjt_ac_->waitForServer(ros::Duration(10)) )
    {
        ROS_ERROR("Timeout FollowJointTrajectory client for robot %s", robot_name_.c_str());
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    std::vector<double> final_config = goal.trajectory.points.back().positions;
//    fjt_ac_->sendGoalAndWait(goal);

    fjt_ac_->sendGoal(goal);

    bool running = true;
    ros::Time t0 = ros::Time::now();
    while ( running )
    {
        ros::Duration(0.001).sleep();

        if ( (ros::Time::now()-t0).toSec() >  (goal.trajectory.points.back().time_from_start * trajectory_time_tollerance_).toSec())
        {
            ROS_YELLOW_STREAM("Execution time ends without finish the trajectory");
            running = false;
        }
        std::vector<double> actual_config =  move_group_->getCurrentJointValues();
        if ( std::abs(final_config.at(0) - actual_config.at(0)) < trajectory_joint_tollerance_.at(0) &&
             std::abs(final_config.at(1) - actual_config.at(1)) < trajectory_joint_tollerance_.at(1) &&
             std::abs(final_config.at(2) - actual_config.at(2)) < trajectory_joint_tollerance_.at(2) &&
             std::abs(final_config.at(3) - actual_config.at(3)) < trajectory_joint_tollerance_.at(3) &&
             std::abs(final_config.at(4) - actual_config.at(4)) < trajectory_joint_tollerance_.at(4) &&
             std::abs(final_config.at(5) - actual_config.at(5)) < trajectory_joint_tollerance_.at(5))
        {
            ROS_WHITE_STREAM("Trajectory executed");
            running = false;
        }
    }

//    fjt_ac_->waitForResult();

    control_msgs::FollowJointTrajectoryResultConstPtr result =  fjt_ac_->getResult();

//    if ( !changeConfig(watch_config_) )
//    {
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
//    }

    if ( contact_ )
    {
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    if ( result->error_code != control_msgs::FollowJointTrajectoryResult::SUCCESSFUL )
    {
        switch (result->error_code) {
        case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL :
            ROS_YELLOW_STREAM("Fjt_result: INVALID_GOAL");
            break;
        case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS :
            ROS_YELLOW_STREAM("Fjt_result: INVALID_JOINTS");
            break;
        case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP :
            ROS_YELLOW_STREAM("Fjt_result: OLD_HEADER_TIMESTAMP");
            break;
        case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED :
            ROS_YELLOW_STREAM("Fjt_result: PATH_TOLERANCE_VIOLATED");
            break;
        case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED :
            ROS_YELLOW_STREAM("Fjt_result: GOAL_TOLERANCE_VIOLATED");
            break;
        default:
            ROS_YELLOW_STREAM("Fjt_result: unknown error");
            break;
        }
        ROS_YELLOW_STREAM("/"<<action_name<<"/"<<skill_name<<" return Fail: "<<skills_executer_msgs::SkillExecutionResponse::Fail);
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    ROS_WHITE_STREAM("/"<<action_name<<"/"<<skill_name<<" return Success: "<<skills_executer_msgs::SkillExecutionResponse::Success);
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

} // end namespace skills_executer
