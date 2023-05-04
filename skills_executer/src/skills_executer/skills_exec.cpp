#include <skills_executer/skills_exec.h>

namespace skills_executer
{

SkillsExec::SkillsExec(const ros::NodeHandle & n) : n_(n)
{
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("/target_cart_twist",1);
    ur_script_command_pub_ = n_.advertise<std_msgs::String>("/ur10e_hw/script_command",10);

    ur_program_stop_clnt_ = n_.serviceClient<std_srvs::Trigger>("/ur10e_hw/dashboard/stop");
    ROS_YELLOW_STREAM("Waiting for "<<ur_program_stop_clnt_.getService());
    ur_program_stop_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");
    ur_program_start_clnt_ = n_.serviceClient<std_srvs::Trigger>("/ur10e_hw/dashboard/play");
    ROS_YELLOW_STREAM("Waiting for "<<ur_program_start_clnt_.getService());
    ur_program_start_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");
    ur_program_state_clnt_ = n_.serviceClient<ur_dashboard_msgs::GetProgramState>("/ur10e_hw/dashboard/program_state");
    ROS_YELLOW_STREAM("Waiting for "<<ur_program_state_clnt_.getService());
    ur_program_state_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");

    board_localization_clnt_ = n_.serviceClient<std_srvs::Trigger>("/robothon2023/board_localization");
    ROS_YELLOW_STREAM("Waiting for "<<board_localization_clnt_.getService());
    board_localization_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");
    circuit_localization_clnt_ = n_.serviceClient<std_srvs::Trigger>("/robothon2023/circuit_localization");
    ROS_YELLOW_STREAM("Waiting for "<<circuit_localization_clnt_.getService());
    circuit_localization_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");
    display_localization_clnt_ = n_.serviceClient<std_srvs::Trigger>("/robothon2023/screen_target");
    ROS_YELLOW_STREAM("Waiting for "<<display_localization_clnt_.getService());
    display_localization_clnt_.waitForExistence();
    display_localization_init_clnt_ = n_.serviceClient<std_srvs::Trigger>("/robothon2023/screen_target_init");
    ROS_YELLOW_STREAM("Waiting for "<<display_localization_init_clnt_.getService());
    display_localization_init_clnt_.waitForExistence();

    digit_screen_reading_clnt_ = n_.serviceClient<std_srvs::Trigger>("/robothon2023/screen_digits");
    ROS_YELLOW_STREAM("Waiting for "<<digit_screen_reading_clnt_.getService());
    digit_screen_reading_clnt_.waitForExistence();

    ROS_YELLOW_STREAM("Connection ok");

    std::string wrench_topic = "/ur_10/wrench";

    wrench_sub_ = std::make_shared<ros_helper::SubscriptionNotifier<geometry_msgs::WrenchStamped>>(n_, wrench_topic, 10);
    js_sub_ = std::make_shared<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>>(n_, "/joint_states", 10);

    skill_exec_srv_ = n_.advertiseService("/skills_exec/execute_skill", &SkillsExec::skillsExecution, this);

    start_config_clnt_ = n_.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
    ROS_YELLOW_STREAM("Waiting for "<<start_config_clnt_.getService());
    start_config_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");

    get_ik_clnt_ = n_.serviceClient<ik_solver_msgs::GetIk>("/manipulator/get_ik");
    ROS_YELLOW_STREAM("Waiting for "<<get_ik_clnt_.getService());
    get_ik_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");

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

    std_srvs::Trigger stop_program_msg;
    ur_program_stop_clnt_.call(stop_program_msg);
    //    end

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

//    std::thread wrench_thread(&SkillsExec::maxWrenchCalculation, this);

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
    else if ( !skill_type.compare(ur_load_program_) )
    {
        res.result = urLoadProgram(req.action_name, req.skill_name);
    }
    else if ( !skill_type.compare(move_to_type_) )
    {
        res.result = move_to(req.action_name, req.skill_name, 0);
//        res.result = follow_joint_trj(req.action_name, req.skill_name, false);
    }
    else if ( !skill_type.compare(linear_move_to_type_) )
    {
        res.result = move_to(req.action_name, req.skill_name, 1);
//        res.result = follow_joint_trj(req.action_name, req.skill_name, true);
    }
    else if ( !skill_type.compare(linear_move_type_) )
    {
        res.result = move_to(req.action_name, req.skill_name, 2);
//        res.result = follow_joint_trj(req.action_name, req.skill_name, true);
    }
    else if ( !skill_type.compare(joint_move_to_type_) )
    {
        res.result = joint_move_to(req.action_name, req.skill_name);
    }
    else if(std::find(ur_type_.begin(), ur_type_.end(), skill_type) != ur_type_.end()) {
        res.result = urScriptCommandExample(req.action_name, req.skill_name, skill_type);
    }
    else if( !skill_type.compare(ur_circula_point_type_) ) {
        res.result = three_circular_point_calculation(req.action_name, req.skill_name);
    }
    else if( !skill_type.compare(ur_linear_move_type_) ) {
        res.result = ur_linear_move(req.action_name, req.skill_name);
    }
    else if( !skill_type.compare(ur_move_to_type_) ) {
        res.result = ur_move_to(req.action_name, req.skill_name);
    }
    else if( !skill_type.compare(board_localization_type_) ) {
        res.result = board_localization();
    }
    else if( !skill_type.compare(circuit_localization_type_) ) {
        res.result = circuit_localization();
    }
    else if( !skill_type.compare(display_localization_type_) ) {
        res.result = display_localization();
    }
    else if( !skill_type.compare(display_localization_init_type_) ) {
        res.result = display_localization_init();
    }
    else if( !skill_type.compare(digit_screen_reading_type_) ) {
        res.result = digit_screen_reading();
    }
    else if( !skill_type.compare(ur_movej_type_) ) {
        res.result = ur_movej(req.action_name, req.skill_name);
    }
    else if( !skill_type.compare(pose_publication_type_) ) {
        res.result = pose_publication(req.action_name, req.skill_name);
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
        setParam(req.action_name, "fail", 1);
        ROS_YELLOW_STREAM("Set /"<<req.action_name<<"/fail: "<<1);
    }

    double duration = ros::Time::now().toSec() - initial_time.toSec();
    total_duration = total_duration + duration;
    setParam(req.action_name,"duration",total_duration);

    end_force_thread_ = true;

//    wrench_thread.join();

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
            if (!getParam(req.action_name, "tollerance", place_tollerance))
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
    parallel_gripper_move_clnt_.waitForExistence();
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

    std::string param_name = "/" + current_grasped_object_ + "/attached";

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
            if ( actual_js.position.at(index - actual_js.name.begin()) < closed_gripper_position_ + gripper_tollerance_ && actual_js.position.at(index - actual_js.name.begin()) > closed_gripper_position_ - gripper_tollerance_ )
            {
                setParam(action_name,skill_name,"fail",1);
                setParam(action_name,"fail",1);
                std::string param_name = "/" + current_grasped_object_ + "/attached";
                n_.setParam(param_name,false);
                ROS_YELLOW_STREAM("No object in the gripper. Current position: "<<actual_js.position.at(index - actual_js.name.begin()));
                return skills_executer_msgs::SkillExecutionResponse::Fail;
            }
        }

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
            n_.setParam(links_param_name,gripper_touch_links_);
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

int SkillsExec::robotiqGripperMove(const std::string &action_name, const std::string &skill_name)
{
    ROS_WHITE_STREAM("In robotiqGripperMove");
    ros::ServiceClient gripper_clnt = n_.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");
    ROS_YELLOW_STREAM("Waiting for "<<gripper_clnt.getService() );
    gripper_clnt.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");
    manipulation_msgs::JobExecution gripper_srv;
    bool asinc;

    if (!getParam(action_name, skill_name, "property_id", gripper_srv.request.property_id))
    {
        ROS_RED_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/property_id is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "skill_name", gripper_srv.request.skill_name))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/skill_name is not set, not necessary" );
    }
    if (!getParam(action_name, skill_name, "tool_id", gripper_srv.request.tool_id))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/tool_id is not set, not necessary" );
    }
    if (!getParam(action_name, skill_name, "asinc", asinc))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/asinc is not set, not necessary, considered False" );
        asinc = false;
    }
    if (asinc)
    {
      gripper_srv.request.object_id = "asinc";
    }
    else
    {
      gripper_srv.request.object_id = "sinc";
    }

    ROS_GREEN_STREAM("Ros time"<<ros::Time::now().toSec());
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

    if ( !changeConfig(watch_config_) )
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
            tf_listener_.lookupTransform( gripper_frame_, target_TF, ros::Time(0), gripper_to_goal_frame_transform);
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
        relative_pose.header.frame_id    = gripper_frame_;

        rel_move_goal.target_angular_velocity = target_angular_velocity;
        rel_move_goal.target_linear_velocity = target_linear_velocity;
        rel_move_goal.relative_pose = relative_pose;

        ROS_WHITE_STREAM("Goal:");
        ROS_WHITE_STREAM("Frame: "<<rel_move_goal.relative_pose.header.frame_id);
        ROS_WHITE_STREAM("Position: ["<<rel_move_goal.relative_pose.pose.position.x<<","<<rel_move_goal.relative_pose.pose.position.y<<","<<rel_move_goal.relative_pose.pose.position.z<<"]");
        ROS_WHITE_STREAM("Quaternion: ["<<rel_move_goal.relative_pose.pose.orientation.x<<","<<rel_move_goal.relative_pose.pose.orientation.y<<","<<rel_move_goal.relative_pose.pose.orientation.z<<","<<rel_move_goal.relative_pose.pose.orientation.w<<"]");
        ROS_WHITE_STREAM("Velocity: lin "<<rel_move_goal.target_linear_velocity<<", rot "<<rel_move_goal.target_angular_velocity);
    }

    if ( !changeConfig(cart_pos_type_) )
    {
        ROS_RED_STREAM("/"<<action_name<<"/"<<skill_name<<" return ProblemConfManager: "<<skills_executer_msgs::SkillExecutionResponse::ProblemConfManager);
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
    }

    relative_move_action_->waitForServer();
    relative_move_action_->sendGoalAndWait(rel_move_goal);

    if ( !changeConfig(watch_config_) )
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

    if ( !changeConfig(watch_config_) )
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
        Eigen::Affine3d T_origin_link_goal = T_origin_goal_gripper* T_relative * T_gripper_link_;
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
                if ( !changeConfig(watch_config_) )
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

    if ( !changeConfig(watch_config_) )
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

bool SkillsExec::changeConfig(std::string config_name)
{
    configuration_msgs::StartConfiguration start_config_srv;
    start_config_srv.request.start_configuration = config_name;
    start_config_srv.request.strictness = 1;

    ROS_YELLOW_STREAM("Waiting for "<<start_config_clnt_.getService());
    start_config_clnt_.waitForExistence();
    ROS_YELLOW_STREAM("Connection ok");

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

    ROS_WHITE_STREAM("Controller "<<config_name<<" started.");

//    ros::Duration(0.1).sleep();
    return true;
}

int SkillsExec::reset_ur10e_ft_sensor()
{
    ROS_WHITE_STREAM("In reset_ur10e_ft_sensor");
    ros::ServiceClient reset_force_sensor_clnt = n_.serviceClient<std_srvs::Trigger>("/ur10e_hw/zero_ftsensor");
    reset_force_sensor_clnt.waitForExistence();
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
    ros::ServiceClient reset_force_sensor_clnt = n_.serviceClient<pybullet_utils::SensorReset>("/pybullet_sensor_reset");
    reset_force_sensor_clnt.waitForExistence();
    pybullet_utils::SensorReset zero_srv;
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

void SkillsExec::maxWrenchCalculation()
{
    reset_pybullet_ft_sensor();
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
//            if ( actual_js.position.at(index - actual_js.name.begin()) < closed_gripper_position_ + gripper_tollerance_ && actual_js.position.at(index - actual_js.name.begin()) > closed_gripper_position_ - gripper_tollerance_ )
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

    //    start current ur program
    std_srvs::Trigger start_program_msg;
    ur_dashboard_msgs::GetProgramState program_state_msg;
    ur_program_start_clnt_.call(start_program_msg);

    ROS_GREEN_STREAM("Waiting for start current program");
    while (true)
    {
        ur_program_state_clnt_.call(program_state_msg);
        if (program_state_msg.response.state.state == "PLAYING")
        {
            break;
        }
        ros::Duration(0.01).sleep();
        ur_program_start_clnt_.call(start_program_msg);
    }


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
    tf::Transform transform;

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
        }
        else
        {
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
        }
        else
        {
            ROS_WHITE_STREAM("  relative_orientation: ["<<relative_quaternion.at(0)<<","<<relative_quaternion.at(1)<<","<<relative_quaternion.at(2)<<","<<relative_quaternion.at(3)<<"]");
        }
    }
    transform.setOrigin( tf::Vector3( relative_position.at(0),relative_position.at(1),relative_position.at(2) ) );
    transform.setRotation( tf::Quaternion( relative_quaternion.at(0),relative_quaternion.at(1),relative_quaternion.at(2),relative_quaternion.at(3) ) );

    std::string TF_name = target_TF;
    TF_name.append("_goal");
    tf_br_.sendTransform( tf::StampedTransform(transform, ros::Time::now(), target_TF, TF_name) );

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
        if ( !changeConfig(watch_config_) )
        {
            ROS_RED_STREAM("Problem with configuration manager");
            return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;
        }
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }

    moveit::core::MoveItErrorCode move_result = move_group_->execute(moveit_plan_);

    if ( !changeConfig(watch_config_) )
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

int SkillsExec::three_circular_point_calculation(const std::string &action_name, const std::string &skill_name)
{
    std::string act_name, sk_name;
    if (!getParam(action_name, skill_name, "action_name", act_name))
    {
        ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/action_name is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    if (!getParam(action_name, skill_name, "skill_name", sk_name))
    {
        ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/skill_name is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    tf::Vector3 pos1, pos2;
    tf::Quaternion quat1, quat2;
    double radius = 0.065;

    tf::StampedTransform pose1_in_gripper_transform, pose1_in_base_transform;
    Eigen::Affine3d T_pose1_in_gripper, T_pose1_in_base;
    tf::StampedTransform pose2_in_gripper_transform, pose2_in_base_transform;
    Eigen::Affine3d T_pose2_in_gripper, T_pose2_in_base;

    pos1.setX(0.0);
    pos1.setY(-(radius * (1 - cos(M_PI/4))));
    pos1.setZ(-radius * sin(M_PI/4));
    quat1.setX(0.0);
    quat1.setY(0.0);
    quat1.setZ(0.0);
    quat1.setW(1.0);

    pose1_in_gripper_transform.setOrigin(pos1);
    pose1_in_gripper_transform.setRotation(quat1);
    ROS_INFO("A10");

    pos2.setX(0.0);
    pos2.setY(-radius * (1 - cos(3*M_PI/4)));
    pos2.setZ(-radius * sin(3*M_PI/4));
    quat2.setX(0.0);
    quat2.setY(0.0);
    quat2.setZ(0.0);
    quat2.setW(1.0);

    pose2_in_gripper_transform.setOrigin(pos2);
    pose2_in_gripper_transform.setRotation(quat2);
    ROS_INFO("A11");

    tf::StampedTransform pose0_in_base_transform;
    bool ok = false;
    while ( !ok )
    {
        try
        {
            tf_listener_.lookupTransform( "base", "real_tool", ros::Time(0), pose0_in_base_transform);
            ok = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    Eigen::Affine3d T_pose0_in_base;
    tf::transformTFToEigen( pose0_in_base_transform, T_pose0_in_base);
    tf::transformTFToEigen( pose1_in_gripper_transform, T_pose1_in_gripper);
    tf::transformTFToEigen( pose2_in_gripper_transform, T_pose2_in_gripper);
    T_pose1_in_base = T_pose0_in_base * T_pose1_in_gripper;
    T_pose2_in_base = T_pose0_in_base * T_pose2_in_gripper;

    tf::transformEigenToTF(T_pose1_in_base, pose1_in_base_transform);
    tf::transformEigenToTF(T_pose2_in_base, pose2_in_base_transform);

//    Ora si settano i parametri della skill che ci interessa
    tf::Vector3 rot0 = pose0_in_base_transform.getRotation().getAngle() * pose0_in_base_transform.getRotation().getAxis();
    tf::Vector3 rot1 = pose1_in_base_transform.getRotation().getAngle() * pose1_in_base_transform.getRotation().getAxis();
    tf::Vector3 rot2 = pose2_in_base_transform.getRotation().getAngle() * pose2_in_base_transform.getRotation().getAxis();

    setParam(act_name,sk_name,"POS_START_X",pose0_in_base_transform.getOrigin().getX());
    setParam(act_name,sk_name,"POS_START_Y",pose0_in_base_transform.getOrigin().getY());
    setParam(act_name,sk_name,"POS_START_Z",pose0_in_base_transform.getOrigin().getZ());

    setParam(act_name,sk_name,"POS_VIA_X",  pose1_in_base_transform.getOrigin().getX());
    setParam(act_name,sk_name,"POS_VIA_Y",  pose1_in_base_transform.getOrigin().getY());
    setParam(act_name,sk_name,"POS_VIA_Z",  pose1_in_base_transform.getOrigin().getZ());

    setParam(act_name,sk_name,"POS_END_X",  pose2_in_base_transform.getOrigin().getX());
    setParam(act_name,sk_name,"POS_END_Y",  pose2_in_base_transform.getOrigin().getY());
    setParam(act_name,sk_name,"POS_END_Z",  pose2_in_base_transform.getOrigin().getZ());

    setParam(act_name,sk_name,"ROT_START_X", rot0.getX());
    setParam(act_name,sk_name,"ROT_START_Y", rot0.getY());
    setParam(act_name,sk_name,"ROT_START_Z", rot0.getZ());

    setParam(act_name,sk_name,"ROT_VIA_X", rot1.getX());
    setParam(act_name,sk_name,"ROT_VIA_Y", rot1.getY());
    setParam(act_name,sk_name,"ROT_VIA_Z", rot1.getZ());

    setParam(act_name,sk_name,"ROT_END_X", rot2.getX());
    setParam(act_name,sk_name,"ROT_END_Y", rot2.getY());
    setParam(act_name,sk_name,"ROT_END_Z", rot2.getZ());

//    tf_br_.sendTransform( tf::StampedTransform(pose0_in_base_transform, ros::Time::now(), "base", "pose0") );
//    tf_br_.sendTransform( tf::StampedTransform(pose1_in_base_transform, ros::Time::now(), "base", "pose1") );
//    tf_br_.sendTransform( tf::StampedTransform(pose2_in_base_transform, ros::Time::now(), "base", "pose2") );

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::urScriptCommandExample(const std::string &action_name, const std::string &skill_name, const std::string &skill_type)
{
    changeConfig("watch");
    ros_helper::SubscriptionNotifier<ur_msgs::IOStates> io_state_sub(n_, "/ur10e_hw/io_states", 10);

//    stop current ur program
    std_srvs::Trigger stop_program_msg;
    ur_program_stop_clnt_.call(stop_program_msg);

    ROS_GREEN_STREAM("Waiting for stop current program");
    ur_dashboard_msgs::GetProgramState program_state_msg;
    while (true)
    {
      ur_program_state_clnt_.call(program_state_msg);
      if (program_state_msg.response.state.state == "STOPPED")
      {
        break;
      }
      ros::Duration(0.01).sleep();
    }

//    read all necessary params
    std::map<std::string,double> params;

    for ( std::string param_name: skill_params_names_[skill_type] )
    {
        double param_value;
        if (!getParam(action_name, skill_name, param_name, param_value))
        {
            ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/"<<param_name<<" is not set" );
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        params.insert( std::make_pair(param_name,param_value) );
    }
//

    std::string file_name = skill_type + ".script";

    std::string path = ros::package::getPath("robothon2023_tree");
    path.append("/ur_script/");
    path.append(file_name);

    std::ifstream read_file;
    read_file.open(path);
    if (!read_file)
    {
        ROS_ERROR_STREAM("Unable to open file "<<path);
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    std::string line_text;
    std::string full_text;
    while (std::getline (read_file, line_text)) {
        full_text.append(line_text);
        full_text.append("\n");
    }
//    ROS_GREEN_STREAM(full_text.c_str());
//    change all "param_name" with their value
    std::size_t index1,index2;
    for ( std::string param_name: skill_params_names_[skill_type] )
    {
        index1 = full_text.find(param_name);
        index2 = full_text.rfind(param_name);
        if ( index1 != std::string::npos && index1 == index2 )
        {
            full_text.replace(index1, param_name.length(), std::to_string(params[param_name]));
        }
        else if ( index1 == std::string::npos )
        {
            ROS_ERROR_STREAM("No param "<<param_name<<" in the script");
            return skills_executer_msgs::SkillExecutionResponse::Error;
        }
        else
        {
            ROS_ERROR_STREAM("Multi params "<<param_name<<" in the script");
            return skills_executer_msgs::SkillExecutionResponse::Error;
        }
    }
//
//    ROS_GREEN_STREAM(full_text.c_str());

    std_msgs::String str_msg;
    str_msg.data = full_text;

    ur_script_command_pub_.publish(str_msg);

    ROS_RED_STREAM("Waiting script start");
    ros::Time start_time = ros::Time::now();
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
      if ( (ros::Time::now().toSec() - start_time.toSec()) > 1.0 )
      {
        break;
      }
    }

    ROS_RED_STREAM("Waiting script end");
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (!io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
    }

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::ur_linear_move(const std::string &action_name, const std::string &skill_name)
{
    ros_helper::SubscriptionNotifier<ur_msgs::IOStates> io_state_sub(n_, "/ur10e_hw/io_states", 10);
    //ik_solver::UrIkSolver ur_solver()
//    stop current ur program
//    std_srvs::Trigger stop_program_msg;
//    ur_program_stop_clnt_.call(stop_program_msg);

    ROS_GREEN_STREAM("Waiting for stop current program");
    ur_dashboard_msgs::GetProgramState program_state_msg;
    while (true)
    {
      ur_program_state_clnt_.call(program_state_msg);
      if (program_state_msg.response.state.state == "STOPPED")
      {
        break;
      }
      ros::Duration(0.01).sleep();
    }
    ROS_GREEN_STREAM("Start movel");


    std::string reference_frame, tool_frame;
    if (!getParam(action_name, skill_name, "reference_frame", reference_frame))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/reference_frame is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "tool_frame", tool_frame))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/tool_frame is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    double velocity, acceleration;
    if (!getParam(action_name, skill_name, "velocity", velocity))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/velocity is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "acceleration", acceleration))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/acceleration is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    int n_point;
    if (!getParam(action_name, skill_name, "n_point", n_point))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/n_point is not set, default value: 1" );
        n_point = 1;
    }

    std::vector<double> traslation, rotation;
    tf::Vector3 tras;
    tf::Quaternion rot;
    if (!getParam(action_name, skill_name, "traslation", traslation))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/traslation is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (traslation.size() != 3)
    {
      ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/traslation size is not 3" );
      return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    tras.setX(traslation[0]);
    tras.setY(traslation[1]);
    tras.setZ(traslation[2]);
    ROS_GREEN_STREAM("Traslation: ["<<traslation.at(0)<<","<<traslation.at(1)<<","<<traslation.at(2)<<"]");

    if (!getParam(action_name, skill_name, "rotation", rotation))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/rotation is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (rotation.size() != 4)
    {
      ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/rotation size is not 4" );
      return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    ROS_GREEN_STREAM("Rotation: ["<<rotation.at(0)<<","<<rotation.at(1)<<","<<rotation.at(2)<<","<<rotation.at(3)<<"]");

    rot.setX(rotation[0]);
    rot.setY(rotation[1]);
    rot.setZ(rotation[2]);
    rot.setW(rotation[3]);

    std::vector<double> v1 = {0,0,0};
    std::vector<double> v2 = {0,0,0,1};

    if ( traslation == v1 && rotation == v2 )
    {
      return skills_executer_msgs::SkillExecutionResponse::Success;
    }

    tf::Vector3 delta_tras = tras / n_point;
    tf::Quaternion delta_rot;
    double rot_x, rot_y, rot_z;
    rot_x = rot.getAngle() * rot.getAxis().getX();
    rot_y = rot.getAngle() * rot.getAxis().getY();
    rot_z = rot.getAngle() * rot.getAxis().getZ();
    ROS_GREEN_STREAM("Rot: ["<<rot_x<<","<<rot_y<<","<<rot_z<<"]");

    double delta_rot_x = rot_x / n_point;
    double delta_rot_y = rot_y / n_point;
    double delta_rot_z = rot_z / n_point;
    delta_rot.setRPY(delta_rot_x,delta_rot_y,delta_rot_z);

    if ( (rot_x != 0 && rot_y != 0) || (rot_x != 0 && rot_z != 0) || (rot_y != 0 && rot_z != 0) )
    {
        ROS_RED_STREAM("More than one rotation is different from 0");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    tf::StampedTransform base_to_reference_transform, reference_to_tool_transform, tool_to_real_tool_trasform;
    ros::Time time_now = ros::Time::now();
    try
    {
        tf_listener_.lookupTransform( "base", reference_frame, ros::Time(0), base_to_reference_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    try
    {
        tf_listener_.lookupTransform( reference_frame, tool_frame, ros::Time(0), reference_to_tool_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    try
    {
        tf_listener_.lookupTransform( tool_frame, "real_tool", ros::Time(0), tool_to_real_tool_trasform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
//    ROS_GREEN_STREAM("Time_now: "<<time_now.toSec());
    ROS_GREEN_STREAM("Time_now - base_to_reference_transform: "<<time_now.toSec()-base_to_reference_transform.stamp_.toSec());
    ROS_GREEN_STREAM("Time_now - reference_to_tool_transform: "<<time_now.toSec()-reference_to_tool_transform.stamp_.toSec());
    ROS_GREEN_STREAM("Time_now - tool_to_real_tool_trasform: "<<time_now.toSec()-tool_to_real_tool_trasform.stamp_.toSec());

    tf::Vector3 null_vec;
    null_vec.setX(0);
    null_vec.setY(0);
    null_vec.setZ(0);
    tf::Quaternion null_quat;
    null_quat.setX(0);
    null_quat.setY(0);
    null_quat.setZ(0);
    null_quat.setW(1);
    tf::StampedTransform variable_trasform;

    tf::StampedTransform traslation_reference_to_tool_transform, traslation_movement_transform, rotation_movement_transform, rotation_reference_to_tool_transform;
    traslation_reference_to_tool_transform.setOrigin(reference_to_tool_transform.getOrigin());
    traslation_reference_to_tool_transform.setRotation(null_quat);
//    movement_transform.setOrigin(tras);
//    movement_transform.setRotation(rot);
    traslation_movement_transform.setOrigin(delta_tras);
    traslation_movement_transform.setRotation(null_quat);
    rotation_movement_transform.setOrigin(null_vec);
    rotation_movement_transform.setRotation(delta_rot);
    rotation_reference_to_tool_transform.setOrigin(null_vec);
    rotation_reference_to_tool_transform.setRotation(reference_to_tool_transform.getRotation());

    Eigen::Affine3d T_base_to_reference, T_traslation_ref_to_tool, T_traslation_movement, T_rotation_movement, T_rotation_ref_to_tool, T_tool_to_real_tool;
    tf::transformTFToEigen(base_to_reference_transform, T_base_to_reference);
    tf::transformTFToEigen(traslation_reference_to_tool_transform, T_traslation_ref_to_tool);
    tf::transformTFToEigen(traslation_movement_transform, T_traslation_movement);
    tf::transformTFToEigen(rotation_movement_transform, T_rotation_movement);
    tf::transformTFToEigen(rotation_reference_to_tool_transform, T_rotation_ref_to_tool);
    tf::transformTFToEigen(tool_to_real_tool_trasform, T_tool_to_real_tool);

    std::vector<Eigen::Affine3d> T_base_parallel_tool_poses, T_tool_poses, T_real_tool_poses;
    std::vector<tf::StampedTransform> base_parallel_tool_poses_transform, tool_poses_transform, real_tool_poses_transform;

    T_base_parallel_tool_poses.push_back(T_base_to_reference * T_traslation_ref_to_tool);
    T_tool_poses.push_back(T_base_parallel_tool_poses.back() * T_rotation_ref_to_tool);
    T_real_tool_poses.push_back(T_tool_poses.back() * T_tool_to_real_tool);
    tf::transformEigenToTF(T_base_parallel_tool_poses.back(), variable_trasform);
    base_parallel_tool_poses_transform.push_back(variable_trasform);
    tf::transformEigenToTF(T_tool_poses.back(), variable_trasform);
    tool_poses_transform.push_back(variable_trasform);
    tf::transformEigenToTF(T_real_tool_poses.back(), variable_trasform);
    real_tool_poses_transform.push_back(variable_trasform);
    std::string mesg = "   base_parallel_tool_poses_transform_: ["+
            std::to_string(base_parallel_tool_poses_transform.back().getOrigin().getX())+","+
            std::to_string(base_parallel_tool_poses_transform.back().getOrigin().getY())+","+
            std::to_string(base_parallel_tool_poses_transform.back().getOrigin().getZ())+"] ["+
            std::to_string(base_parallel_tool_poses_transform.back().getRotation().getAngle() * base_parallel_tool_poses_transform.back().getRotation().getAxis().getX())+","+
            std::to_string(base_parallel_tool_poses_transform.back().getRotation().getAngle() * base_parallel_tool_poses_transform.back().getRotation().getAxis().getY())+","+
            std::to_string(base_parallel_tool_poses_transform.back().getRotation().getAngle() * base_parallel_tool_poses_transform.back().getRotation().getAxis().getZ())+"]";
    ROS_GREEN_STREAM(mesg);
    mesg = "   tool_poses_transform_: ["+
            std::to_string(tool_poses_transform.back().getOrigin().getX())+","+
            std::to_string(tool_poses_transform.back().getOrigin().getY())+","+
            std::to_string(tool_poses_transform.back().getOrigin().getZ())+"] ["+
            std::to_string(tool_poses_transform.back().getRotation().getAngle() * tool_poses_transform.back().getRotation().getAxis().getX())+","+
            std::to_string(tool_poses_transform.back().getRotation().getAngle() * tool_poses_transform.back().getRotation().getAxis().getY())+","+
            std::to_string(tool_poses_transform.back().getRotation().getAngle() * tool_poses_transform.back().getRotation().getAxis().getZ())+"]";
    ROS_GREEN_STREAM(mesg);
    mesg = "   real_tool_poses_transform_: ["+
            std::to_string(real_tool_poses_transform.back().getOrigin().getX())+","+
            std::to_string(real_tool_poses_transform.back().getOrigin().getY())+","+
            std::to_string(real_tool_poses_transform.back().getOrigin().getZ())+"] ["+
            std::to_string(real_tool_poses_transform.back().getRotation().getAngle() * real_tool_poses_transform.back().getRotation().getAxis().getX())+","+
            std::to_string(real_tool_poses_transform.back().getRotation().getAngle() * real_tool_poses_transform.back().getRotation().getAxis().getY())+","+
            std::to_string(real_tool_poses_transform.back().getRotation().getAngle() * real_tool_poses_transform.back().getRotation().getAxis().getZ())+"]";
    ROS_GREEN_STREAM(mesg);

    Eigen::Affine3d T_cumulative_rotation_movement = T_rotation_movement;
    for ( int i = 0; i < n_point; i++ )
    {
        T_base_parallel_tool_poses.push_back(T_base_parallel_tool_poses.back() * T_traslation_movement);
        T_tool_poses.push_back(T_base_parallel_tool_poses.back() * T_cumulative_rotation_movement * T_rotation_ref_to_tool);
        T_real_tool_poses.push_back(T_tool_poses.back() * T_tool_to_real_tool);
        tf::transformEigenToTF(T_base_parallel_tool_poses.back(), variable_trasform);
        base_parallel_tool_poses_transform.push_back(variable_trasform);
        tf::transformEigenToTF(T_tool_poses.back(), variable_trasform);
        tool_poses_transform.push_back(variable_trasform);
        tf::transformEigenToTF(T_real_tool_poses.back(), variable_trasform);
        real_tool_poses_transform.push_back(variable_trasform);

        T_cumulative_rotation_movement = T_cumulative_rotation_movement * T_rotation_movement;

//        std::string mesg = "   base_parallel_tool_poses_transform"+std::to_string(i)+": ["+
//                std::to_string(base_parallel_tool_poses_transform.back().getOrigin().getX())+","+
//                std::to_string(base_parallel_tool_poses_transform.back().getOrigin().getY())+","+
//                std::to_string(base_parallel_tool_poses_transform.back().getOrigin().getZ())+"] ["+
//                std::to_string(base_parallel_tool_poses_transform.back().getRotation().getAngle() * base_parallel_tool_poses_transform.back().getRotation().getAxis().getX())+","+
//                std::to_string(base_parallel_tool_poses_transform.back().getRotation().getAngle() * base_parallel_tool_poses_transform.back().getRotation().getAxis().getY())+","+
//                std::to_string(base_parallel_tool_poses_transform.back().getRotation().getAngle() * base_parallel_tool_poses_transform.back().getRotation().getAxis().getZ())+"]";
//        ROS_GREEN_STREAM(mesg);
        std::string mesg = "   tool_poses_transform"+std::to_string(i)+": ["+
                std::to_string(tool_poses_transform.back().getOrigin().getX())+","+
                std::to_string(tool_poses_transform.back().getOrigin().getY())+","+
                std::to_string(tool_poses_transform.back().getOrigin().getZ())+"] ["+
                std::to_string(tool_poses_transform.back().getRotation().getAngle() * tool_poses_transform.back().getRotation().getAxis().getX())+","+
                std::to_string(tool_poses_transform.back().getRotation().getAngle() * tool_poses_transform.back().getRotation().getAxis().getY())+","+
                std::to_string(tool_poses_transform.back().getRotation().getAngle() * tool_poses_transform.back().getRotation().getAxis().getZ())+"]";
        ROS_GREEN_STREAM(mesg);
//        mesg = "   real_tool_poses_transform"+std::to_string(i)+": ["+
//                std::to_string(real_tool_poses_transform.back().getOrigin().getX())+","+
//                std::to_string(real_tool_poses_transform.back().getOrigin().getY())+","+
//                std::to_string(real_tool_poses_transform.back().getOrigin().getZ())+"] ["+
//                std::to_string(real_tool_poses_transform.back().getRotation().getAngle() * real_tool_poses_transform.back().getRotation().getAxis().getX())+","+
//                std::to_string(real_tool_poses_transform.back().getRotation().getAngle() * real_tool_poses_transform.back().getRotation().getAxis().getY())+","+
//                std::to_string(real_tool_poses_transform.back().getRotation().getAngle() * real_tool_poses_transform.back().getRotation().getAxis().getZ())+"]";
//        ROS_GREEN_STREAM(mesg);


        tf_br_.sendTransform( tf::StampedTransform(tool_poses_transform.back(), ros::Time::now(), "base", "tool_n_"+std::to_string(i)) );
        tf_br_.sendTransform( tf::StampedTransform(real_tool_poses_transform.back(), ros::Time::now(), "base", "real_tool_n_"+std::to_string(i)) );
    }

    std::string file_name = "ur_script.script";

    std::string path = ros::package::getPath("robothon2023_tree");
    path.append("/ur_script/");
    path.append(file_name);

    std::ifstream read_file;
    read_file.open(path);
    if (!read_file)
    {
        ROS_ERROR_STREAM("Unable to open file "<<path);
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    std::string line_text;
    std::string full_text;
    while (std::getline (read_file, line_text)) {
        full_text.append(line_text);
        full_text.append("\n");
    }

    std::string message = "set_standard_digital_out(7, True)\n";

    if ( n_point == 1 )
    {
        for ( int i = 1; i < n_point+1; i++ )
        {

            message = message + "movel(p["+
                    std::to_string(real_tool_poses_transform.at(i).getOrigin().getX())+","+
                    std::to_string(real_tool_poses_transform.at(i).getOrigin().getY())+","+
                    std::to_string(real_tool_poses_transform.at(i).getOrigin().getZ())+","+
                    std::to_string(real_tool_poses_transform.at(i).getRotation().getAngle() * real_tool_poses_transform.at(i).getRotation().getAxis().getX())+","+
                    std::to_string(real_tool_poses_transform.at(i).getRotation().getAngle() * real_tool_poses_transform.at(i).getRotation().getAxis().getY())+","+
                    std::to_string(real_tool_poses_transform.at(i).getRotation().getAngle() * real_tool_poses_transform.at(i).getRotation().getAxis().getZ())+"], a="+
                    std::to_string(acceleration)+", v="+
                    std::to_string(velocity)+ ")\n";
        }
    }
    else
    {
        for ( int i = 1; i < n_point+1; i++ )
        {
            message = message + "movej(p["+
                    std::to_string(real_tool_poses_transform.at(i).getOrigin().getX())+","+
                    std::to_string(real_tool_poses_transform.at(i).getOrigin().getY())+","+
                    std::to_string(real_tool_poses_transform.at(i).getOrigin().getZ())+","+
                    std::to_string(real_tool_poses_transform.at(i).getRotation().getAngle() * real_tool_poses_transform.at(i).getRotation().getAxis().getX())+","+
                    std::to_string(real_tool_poses_transform.at(i).getRotation().getAngle() * real_tool_poses_transform.at(i).getRotation().getAxis().getY())+","+
                    std::to_string(real_tool_poses_transform.at(i).getRotation().getAngle() * real_tool_poses_transform.at(i).getRotation().getAxis().getZ())+"], a="+
                    std::to_string(acceleration)+", v="+
                    std::to_string(velocity);
            if ( i == 0 || i == n_point)
            {
                message = message + ")\n";
            }
            else{
                message = message + ", r=0.005)\n";
            }
        }
    }
    message = message + "set_standard_digital_out(7, False)";

    ROS_GREEN_STREAM(message);

    std::size_t index1,index2;
    std::string param_name = "SCRIPT";
    index1 = full_text.find(param_name);
    index2 = full_text.rfind(param_name);
    if ( index1 != std::string::npos && index1 == index2 )
    {
        full_text.replace(index1, param_name.length(), message);
    }
    else if ( index1 == std::string::npos )
    {
        ROS_ERROR_STREAM("No param "<<param_name<<" in the script");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
        ROS_ERROR_STREAM("Multi params "<<param_name<<" in the script");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    std_msgs::String str_msg;
    str_msg.data = full_text;

    ur_script_command_pub_.publish(str_msg);

    ROS_RED_STREAM("Waiting script start");
    ros::Time start_time = ros::Time::now();
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
      if ( (ros::Time::now().toSec() - start_time.toSec()) > 1.0 )
      {
        break;
      }
    }

    ROS_RED_STREAM("Waiting script end");
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (!io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
    }

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::ur_move_to(const std::string &action_name, const std::string &skill_name)
{
    ros_helper::SubscriptionNotifier<ur_msgs::IOStates> io_state_sub(n_, "/ur10e_hw/io_states", 10);

//    stop current ur program
    std_srvs::Trigger stop_program_msg;
    ur_program_stop_clnt_.call(stop_program_msg);

    ROS_GREEN_STREAM("Waiting for stop current program");
    ur_dashboard_msgs::GetProgramState program_state_msg;
    while (true)
    {
      ur_program_state_clnt_.call(program_state_msg);
      if (program_state_msg.response.state.state == "STOPPED")
      {
        break;
      }
      ros::Duration(0.01).sleep();
    }
    ROS_GREEN_STREAM("Start movel");


    std::string goal_frame, tool_frame;
    bool ik;
    if (!getParam(action_name, skill_name, "goal_frame", goal_frame))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/goal_frame is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "tool_frame", tool_frame))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/tool_frame is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    double velocity, acceleration;
    if (!getParam(action_name, skill_name, "velocity", velocity))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/velocity is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "acceleration", acceleration))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/acceleration is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "ik", ik))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/ik is not set, default false" );
        ik = false;
    }
    tf::StampedTransform base_to_goal_frame_transform, tool_frame_to_real_tool_transform, base_to_real_tool_goal_frame_transform;

    try
    {
        tf_listener_.lookupTransform( "base", goal_frame, ros::Time(0), base_to_goal_frame_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    try
    {
        tf_listener_.lookupTransform( tool_frame, "real_tool", ros::Time(0), tool_frame_to_real_tool_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    ik_solver_msgs::GetIk get_ik_msg;
    if (ik)
    {
      ik_solver_msgs::Configuration ik_configuration;

      ik_configuration.configuration = move_group_->getCurrentJointValues();

      get_ik_msg.request.tf_name = goal_frame;
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

      if ( get_ik_msg.response.solution.configurations.at(0).configuration.at(5) > M_PI)
      {
        get_ik_msg.response.solution.configurations.at(0).configuration.at(5) = get_ik_msg.response.solution.configurations.at(0).configuration.at(5) - (2*M_PI);
      }
      else if ( get_ik_msg.response.solution.configurations.at(0).configuration.at(5) < -M_PI)
      {
        get_ik_msg.response.solution.configurations.at(0).configuration.at(5) = get_ik_msg.response.solution.configurations.at(0).configuration.at(5) + (2*M_PI);
      }

    }

    Eigen::Affine3d T_base_to_goal_frame, T_tool_frame_to_real_tool, T_base_to_real_tool_goal_frame;
    tf::transformTFToEigen(base_to_goal_frame_transform, T_base_to_goal_frame);
    tf::transformTFToEigen(tool_frame_to_real_tool_transform, T_tool_frame_to_real_tool);
    T_base_to_real_tool_goal_frame = T_base_to_goal_frame * T_tool_frame_to_real_tool;
    tf::transformEigenToTF(T_base_to_real_tool_goal_frame, base_to_real_tool_goal_frame_transform);

    std::string message = "set_standard_digital_out(7, True)\n";
    if ( ik )
    {
      message = message + "  movej(["+
          std::to_string(get_ik_msg.response.solution.configurations.at(0).configuration.at(0))+","+
          std::to_string(get_ik_msg.response.solution.configurations.at(0).configuration.at(1))+","+
          std::to_string(get_ik_msg.response.solution.configurations.at(0).configuration.at(2))+","+
          std::to_string(get_ik_msg.response.solution.configurations.at(0).configuration.at(3))+","+
          std::to_string(get_ik_msg.response.solution.configurations.at(0).configuration.at(4))+","+
          std::to_string(get_ik_msg.response.solution.configurations.at(0).configuration.at(5))+"], a="+
          std::to_string(acceleration)+", v="+
          std::to_string(velocity)+ ")\n";
    }
    else
    {
      message = message + "  movel(p["+
          std::to_string(base_to_real_tool_goal_frame_transform.getOrigin().getX())+","+
          std::to_string(base_to_real_tool_goal_frame_transform.getOrigin().getY())+","+
          std::to_string(base_to_real_tool_goal_frame_transform.getOrigin().getZ())+","+
          std::to_string(base_to_real_tool_goal_frame_transform.getRotation().getAngle() * base_to_real_tool_goal_frame_transform.getRotation().getAxis().getX())+","+
          std::to_string(base_to_real_tool_goal_frame_transform.getRotation().getAngle() * base_to_real_tool_goal_frame_transform.getRotation().getAxis().getY())+","+
          std::to_string(base_to_real_tool_goal_frame_transform.getRotation().getAngle() * base_to_real_tool_goal_frame_transform.getRotation().getAxis().getZ())+"], a="+
          std::to_string(acceleration)+", v="+
          std::to_string(velocity)+ ")\n";
    }
    message = message + "  set_standard_digital_out(7, False)";

    ROS_GREEN_STREAM(message);

    if( !fill_the_script(message) )
    {
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    std_msgs::String str_msg;
    str_msg.data = message;

    ROS_GREEN_STREAM("Ros time"<<ros::Time::now().toSec());

    ur_script_command_pub_.publish(str_msg);

    ROS_RED_STREAM("Waiting script start");
    ros::Time start_time = ros::Time::now();
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
      if ( (ros::Time::now().toSec() - start_time.toSec()) > 1.0 )
      {
        break;
      }
    }

    ROS_RED_STREAM("Waiting script end");
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (!io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
    }

    return skills_executer_msgs::SkillExecutionResponse::Success;

}

int SkillsExec::ur_movej(const std::string &action_name, const std::string &skill_name)
{
    ros_helper::SubscriptionNotifier<ur_msgs::IOStates> io_state_sub(n_, "/ur10e_hw/io_states", 10);

//    stop current ur program
    std_srvs::Trigger stop_program_msg;
    ur_program_stop_clnt_.call(stop_program_msg);

    ROS_GREEN_STREAM("Waiting for stop current program");
    ur_dashboard_msgs::GetProgramState program_state_msg;
    while (true)
    {
      ur_program_state_clnt_.call(program_state_msg);
      if (program_state_msg.response.state.state == "STOPPED")
      {
        break;
      }
      ros::Duration(0.01).sleep();
    }
    ROS_GREEN_STREAM("Start movel");

    std::vector<double> joint_target;

    if ( !getParam(action_name, skill_name, "joint_target", joint_target) )
    {
        ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/joint_target is not set.");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if ( joint_target.size() != 6)
    {
        ROS_RED_STREAM("Joint target size is not 6");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    double velocity, acceleration;
    if (!getParam(action_name, skill_name, "velocity", velocity))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/velocity is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "acceleration", acceleration))
    {
        ROS_YELLOW_STREAM("The parameter "<<action_name<<"/"<<skill_name<<"/acceleration is not set" );
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    std::string message = "set_standard_digital_out(7, True)\n";
    message = message + "  movej(["+
            std::to_string(joint_target.at(0))+","+
            std::to_string(joint_target.at(1))+","+
            std::to_string(joint_target.at(2))+","+
            std::to_string(joint_target.at(3))+","+
            std::to_string(joint_target.at(4))+","+
            std::to_string(joint_target.at(5))+"], a="+
            std::to_string(acceleration)+", v="+
            std::to_string(velocity)+ ")\n";
    message = message + "  set_standard_digital_out(7, False)";

    ROS_GREEN_STREAM(message);

    if( !fill_the_script(message) )
    {
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    std_msgs::String str_msg;
    str_msg.data = message;

    ur_script_command_pub_.publish(str_msg);

    ROS_RED_STREAM("Waiting script start");
    ros::Time start_time = ros::Time::now();
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
      if ( (ros::Time::now().toSec() - start_time.toSec()) > 1.0 )
      {
        break;
      }
    }

    ROS_RED_STREAM("Waiting script end");
    while (true)
    {
      if (io_state_sub.isANewDataAvailable())
      {
        if (!io_state_sub.getData().digital_out_states[7].state)
        {
          break;
        }
      }
      ros::Duration(0.001).sleep();
    }

    return skills_executer_msgs::SkillExecutionResponse::Success;

}

int SkillsExec::board_localization()
{
    std_srvs::Trigger msg;

    if ( !board_localization_clnt_.call(msg) )
    {
      ROS_RED_STREAM("Board localization server don't work");
      return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
      if(not msg.response.success)
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::circuit_localization()
{
    std_srvs::Trigger msg;

    if ( !circuit_localization_clnt_.call(msg) )
    {
      ROS_RED_STREAM("Circuit localization server don't work");
      return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
      if(not msg.response.success)
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::display_localization()
{
    std_srvs::Trigger msg;

    if ( !display_localization_clnt_.call(msg) )
    {
      ROS_RED_STREAM("Display localization server don't work");
      return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
      if(not msg.response.success)
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::digit_screen_reading()
{
    std_srvs::Trigger msg;

    if ( !digit_screen_reading_clnt_.call(msg) )
    {
      ROS_RED_STREAM("Digit screen reading server don't work");
      return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
      if(not msg.response.success)
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::display_localization_init()
{
    std_srvs::Trigger msg;

    if ( !display_localization_init_clnt_.call(msg) )
    {
      ROS_RED_STREAM("Display localization init server don't work");
      return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
      if(not msg.response.success)
        return skills_executer_msgs::SkillExecutionResponse::Fail;
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

bool SkillsExec::fill_the_script(std::string &script_string)
{
    std::string line_text, full_text;
    std::size_t index1,index2;
    std::string param_name = "SCRIPT";
    std::string file_name = "ur_script.script";
    std::string path = ros::package::getPath("robothon2023_tree");
    path.append("/ur_script/");
    path.append(file_name);

    std::ifstream read_file;

    read_file.open(path);
    if (!read_file)
    {
        ROS_ERROR_STREAM("Unable to open file "<<path);
        return false;
    }

    while (std::getline (read_file, line_text)) {
        full_text.append(line_text);
        full_text.append("\n");
    }
//    ROS_GREEN_STREAM(full_text.c_str());

    index1 = full_text.find(param_name);
    index2 = full_text.rfind(param_name);
    if ( index1 != std::string::npos && index1 == index2 )
    {
        full_text.replace(index1, param_name.length(), script_string);
    }
    else if ( index1 == std::string::npos )
    {
        ROS_ERROR_STREAM("No param "<<param_name<<" in the script");
        return false;
    }
    else
    {
        ROS_ERROR_STREAM("Multi params "<<param_name<<" in the script");
        return false;
    }
    script_string = full_text;
    return true;
}

int SkillsExec::pose_publication(const std::string &action_name, const std::string &skill_name)
{
    std::string tf_name, target_frame;
    if ( !getParam(action_name, skill_name, "tf_name", tf_name) )
    {
        ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/tf_name is not set.");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if ( !getParam(action_name, skill_name, "target_frame", target_frame) )
    {
        ROS_RED_STREAM("  The parameter "<<action_name<<"/"<<skill_name<<"/tf_name is not set.");
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    tf::StampedTransform transform;
    try
    {
        tf_listener_.lookupTransform( "world", target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
       ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
       return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = transform.frame_id_;
    static_transformStamped.child_frame_id = tf_name;
    static_transformStamped.transform.translation.x = transform.getOrigin().getX();
    static_transformStamped.transform.translation.y = transform.getOrigin().getY();
    static_transformStamped.transform.translation.z = transform.getOrigin().getZ();
    static_transformStamped.transform.rotation.x    = transform.getRotation().getX();
    static_transformStamped.transform.rotation.y    = transform.getRotation().getY();
    static_transformStamped.transform.rotation.z    = transform.getRotation().getZ();
    static_transformStamped.transform.rotation.w    = transform.getRotation().getW();
    static_tf_br_.sendTransform(static_transformStamped);

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

} // end namespace skills_executer
