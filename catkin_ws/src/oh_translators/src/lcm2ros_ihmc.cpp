// Copyright 2015 Maurice Fallon, Vladimir Ivan

#include "lcm2ros_ihmc.hpp"

std::vector<std::string> TrajectoryNames = {"Whole Body", "Left Arm", "Right Arm", "Both Arms"};



LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in, std::string robotName_in):
    lcm_(lcm_in), nh_(nh_in), robotName_(robotName_in)
{
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(model_->getURDFString(), tree))
  {
    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
    exit(-1);
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  if (robotName_.compare("atlas") == 0)  // Atlas has utorso
    chestLinkName_ = "utorso";
  else  // valkyrie has torso
    chestLinkName_ = "torso";
  std::cout << "Using "<< robotName_ << " so expecting chest link called " << chestLinkName_ << "\n";

  // Hard Coded Parameters:
  // Conservative values for real Valkyrie, using defaults used by IHMC
  planDesiredOffset_ = 1.0;
  // Variable to set what part of a whole body plan gets passed through to Val:
  outputTrajectoryMode_ = TrajectoryMode::wholeBody;
  ROS_ERROR("LCM2ROS Controller TrajectoryMode: %s", TrajectoryNames.at( (int) outputTrajectoryMode_).c_str() );

  ////////////////// Subscriptions and Adverts //////////////////////
  // If pronto is running never send plans like this:
  lcm_->subscribe("WALKING_CONTROLLER_PLAN_REQUEST", &LCM2ROS::footstepPlanHandler, this);
  // COMMITTED_FOOTSTEP_PLAN is creating in Pronto frame and transformed into 
  // BDI/IHMC coordinate frame using BDI_ADJUSTED_FOOTSTEP_PLAN
  lcm_->subscribe("BDI_ADJUSTED_FOOTSTEP_PLAN", &LCM2ROS::footstepPlanBDIModeHandler, this);
  walking_plan_pub_ = nh_.advertise<ihmc_msgs::FootstepDataListMessage>(
      "/ihmc_ros/" + robotName_ + "/control/footstep_list", 10);


  lcm_->subscribe("VAL_COMMAND_PAUSE", &LCM2ROS::pauseHandler, this);
  lcm_->subscribe("STOP_WALKING", &LCM2ROS::stopHandler, this);  // from Director
  pause_pub_ = nh_.advertise<ihmc_msgs::PauseCommandMessage>("/ihmc_ros/" + robotName_ + "/control/pause_footstep_exec",
                                                             10);
  lcm_->subscribe("COMMITTED_PLAN_PAUSE", &LCM2ROS::stopManipHandler, this);  // from Director to stop manipulation plans
  stop_manip_pub_ = nh_.advertise<ihmc_msgs::StopMotionPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/stop_motion", 10);

  // robot plan messages now used, untested
  lcm_->subscribe("COMMITTED_ROBOT_PLAN", &LCM2ROS::robotPlanHandler, this);
  arm_joint_traj2_pub_ = nh_.advertise<ihmc_msgs::ArmJointTrajectoryPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/arm_joint_trajectory", 10);
  whole_body_trajectory_pub_ = nh_.advertise<ihmc_msgs::WholeBodyTrajectoryPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/whole_body_trajectory", 10);


  lcm_->subscribe("DESIRED_NECK_ANGLES", &LCM2ROS::neckPitchHandler, this);
  lcm_->subscribe("DESIRED_HEAD_ORIENTATION", &LCM2ROS::headOrientationHandler, this);
  neck_orientation_pub_ = nh_.advertise<ihmc_msgs::HeadOrientationPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/head_orientation", 10);

  lcm_->subscribe("DESIRED_LEFT_FOOT_POSE", &LCM2ROS::lFootPoseHandler, this);
  lcm_->subscribe("DESIRED_RIGHT_FOOT_POSE", &LCM2ROS::rFootPoseHandler, this);
  // Only one topic for both feet
  foot_pose_pub_ = nh_.advertise<ihmc_msgs::FootPosePacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/foot_pose", 10);


  lcm_->subscribe("VAL_COMMAND_COM_HEIGHT", &LCM2ROS::comHeightHandler, this);
  com_height_pub_ = nh_.advertise<ihmc_msgs::ComHeightPacketMessage>("/ihmc_ros/" + robotName_ + "/control/com_height",
                                                                     10);


  lcm_->subscribe("SCS_API_CONTROL", &LCM2ROS::scsAPIHandler, this);
  scs_api_pub_ = nh_.advertise<std_msgs::String>("/ihmc_ros/" + robotName_ + "/api_command", 10);


  // Subscriptions that handle local variables:
  lcm_->subscribe("IHMC_CONTROL_MODE_COMMAND", &LCM2ROS::ihmcControlModeCommandHandler, this);

  node_ = new ros::NodeHandle();
}



Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw)
{
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0
  // Post DRC Trails: replace these with Eigen's own conversions
  if (((roll == M_PI) && (pitch == 0)) && (yaw == 0))
  {
    return Eigen::Quaterniond(0, 1, 0, 0);
  }
  else if (((pitch == M_PI) && (roll == 0)) && (yaw == 0))
  {
    return Eigen::Quaterniond(0, 0, 1, 0);
  }
  else if (((yaw == M_PI) && (roll == 0)) && (pitch == 0))
  {
    return Eigen::Quaterniond(0, 0, 0, 1);
  }

  double sy = sin(yaw * 0.5);
  double cy = cos(yaw * 0.5);
  double sp = sin(pitch * 0.5);
  double cp = cos(pitch * 0.5);
  double sr = sin(roll * 0.5);
  double cr = cos(roll * 0.5);
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;
  return Eigen::Quaterniond(w, x, y, z);
}

void LCM2ROS::scsAPIHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                            const drc::scs_api_command_t* msg)
{
  std_msgs::String rmsg;
  rmsg.data = msg->command;
  scs_api_pub_.publish(rmsg);
}



void LCM2ROS::comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const ihmc::com_height_packet_message_t* msg)
{
  ROS_ERROR("LCM2ROS got com height");
  ihmc_msgs::ComHeightPacketMessage mout;
  mout.height_offset = msg->height_offset;
  com_height_pub_.publish(mout);
}

void LCM2ROS::pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const ihmc::pause_command_message_t* msg)
{
  ROS_ERROR("LCM2ROS got pause %d", static_cast<int>(msg->pause));
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = msg->pause;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got STOP_WALKING - sending pause=true");
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = true;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got COMMITTED_PLAN_PAUSE - sending manipulate stop");
  ihmc_msgs::StopMotionPacketMessage mout;
  mout.unique_id = msg->utime;
  stop_manip_pub_.publish(mout);
}

void LCM2ROS::neckPitchHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::joint_angles_t* msg)
{
  ROS_ERROR("LCM2ROS got desired neck pitch");

  int lower_neck_pitch_id = -1;
  for (int i = 0; i < msg->num_joints; i++) {
    if (msg->joint_name[i] == "lowerNeckPitch") {
      lower_neck_pitch_id = i;
      break;
    }
  }

  if (lower_neck_pitch_id == -1) {
    ROS_WARN("lowerNeckPitch not in DESIRED_NECK_ANGLES message, ignoring");
    return;
  }

  ihmc_msgs::HeadOrientationPacketMessage mout;
  Eigen::Quaterniond quat = euler_to_quat(0, msg->joint_position[lower_neck_pitch_id], 0);
  mout.trajectory_time = 1;
  mout.orientation.w = quat.w();
  mout.orientation.x = quat.x();
  mout.orientation.y = quat.y();
  mout.orientation.z = quat.z();
  mout.unique_id = msg->utime;
  neck_orientation_pub_.publish(mout);
}

void LCM2ROS::headOrientationHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg)
{
  ROS_ERROR("LCM2ROS got desired desired head orientation");
  ihmc_msgs::HeadOrientationPacketMessage mout;
  mout.trajectory_time = 1;
  mout.orientation.w = msg->orientation[0];
  mout.orientation.x = msg->orientation[1];
  mout.orientation.y = msg->orientation[2];
  mout.orientation.z = msg->orientation[3];
  mout.unique_id = msg->utime;
  neck_orientation_pub_.publish(mout);
}



void filterJointNamesToIHMC(std::vector<std::string> &joint_name)
{
  // Rename these joints to expected values:

  int n_joints = joint_name.size();
  for (int i = 0; i < n_joints; i++)
  {
    // ihmc v3 to mit v3:
    // if (joint_name[i] == "l_arm_shz"){
    //   joint_name[i] = "l_arm_usy";
    // }
    // if (joint_name[i] == "r_arm_shz"){
    //   joint_name[i] = "r_arm_usy";
    // }

    if (joint_name[i] == "l_arm_uwy")
    {
      joint_name[i] = "l_arm_wry";
    }
    if (joint_name[i] == "l_arm_mwx")
    {
      joint_name[i] = "l_arm_wrx";
    }
    if (joint_name[i] == "l_arm_mwx")
    {
      joint_name[i] = "l_arm_wrx";
    }
    if (joint_name[i] == "r_arm_uwy")
    {
      joint_name[i] = "r_arm_wry";
    }
    if (joint_name[i] == "r_arm_mwx")
    {
      joint_name[i] = "r_arm_wrx";
    }

    // ihmc v5 to mit v5:
    if (joint_name[i] == "l_arm_lwy")
    {
      joint_name[i] = "l_arm_wry2";
    }
    if (joint_name[i] == "r_arm_lwy")
    {
      joint_name[i] = "r_arm_wry2";
    }

    if (joint_name[i] == "neck_ay")
    {
      joint_name[i] = "neck_ry";
    }
    if (joint_name[i] == "hokuyo_joint")
    {
      // double output = remainderf( joint_position[i] , M_PI);
      // std::cout << (joint_position[i]) << " "  << output << "\n";
      // joint_position[i] = output;
      // joint_name[i] = "hokuyo_link";
    }
  }
}

void LCM2ROS::sendSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                                std::vector<std::string> input_joint_names, bool is_right)
{
  ihmc_msgs::ArmJointTrajectoryPacketMessage m;
  bool status = getSingleArmPlan(msg, output_joint_names_arm, input_joint_names, is_right, m);
  if (status)
    arm_joint_traj2_pub_.publish(m);
}


double LCM2ROS::getPlanTimeAtWaypoint(int64_t planUtime)
{
  // Convert the input plan timestamp (int64_t into a safe output (double, seconds)
  // including adjusting by a fixed offset which is required to do the difference
  // between the current joint positions (from where the plan begins) and the
  // controller's current desired position references. Used for both arm and pelvis timestamps
  double time_at_waypoint = (double) planDesiredOffset_ + (planUtime * 1E-6);
  return time_at_waypoint;
}



bool LCM2ROS::getSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                               std::vector<std::string> input_joint_names, bool is_right,
                               ihmc_msgs::ArmJointTrajectoryPacketMessage &m)
{
  // Find the indices of the arm joints which we want to extract
  std::vector<int> arm_indices;
  for (size_t i = 0; i < output_joint_names_arm.size(); i++)
  {
    std::string name = output_joint_names_arm[i];
    std::vector<std::string>::iterator it;
    it = find(input_joint_names.begin(), input_joint_names.end(), name);
    int index = std::distance(input_joint_names.begin(), it);
    if (index < input_joint_names.size())
    {
      // std::cout << name << " found in input_joint_names at " << index << '\n';
      arm_indices.push_back(index);
    }
    else
    {
      ROS_ERROR("%s not found in input_joint_names, not sending plan", name.c_str());
      std::cout << name << " not found in input_joint_names, not sending plan\n";
      return false;
    }
  }

  // Fish out the arm indices:
  if (is_right)
  {
    m.robot_side = 1;
  }
  else
  {
    m.robot_side = 0;
  }

  // m.joint_names = output_joint_names_arm;
  for (int i = 0; i < msg->num_states; i++)  // NB: skipping the first sample as it has time = 0
  {
    bot_core::robot_state_t state = msg->plan[i];
    ihmc_msgs::JointTrajectoryPointMessage point;
    int i1 = (i > 0) ? (i - 1) : 0;
    int i2 = i;
    int i3 = (i < msg->num_states - 1) ? (i + 1) : (msg->num_states - 1);
    for (int j = 0; j < arm_indices.size(); j++)
    {
      point.positions.push_back(state.joint_position[arm_indices[j]]);
      double dt1 = (msg->plan[i2].utime - msg->plan[i1].utime) * 1e-6;
      double dt2 = (msg->plan[i3].utime - msg->plan[i2].utime) * 1e-6;
      double dq1 = msg->plan[i2].joint_position[arm_indices[j]] - msg->plan[i1].joint_position[arm_indices[j]];
      double dq2 = msg->plan[i3].joint_position[arm_indices[j]] - msg->plan[i2].joint_position[arm_indices[j]];
      point.velocities.push_back((dt1 * dt2 != 0) ? (dq1 / dt1 * 0.5 + dq2 / dt2 * 0.5) : 0.0);
      // point.accelerations.push_back( 0  );
      // point.effort.push_back( state.joint_effort[ arm_indices[j] ] );

      point.time = getPlanTimeAtWaypoint( state.utime );
//      std::cout << i << ": " << getPlanTimeAtWaypoint(state.utime) << " " << (state.utime - msg->plan[i-1].utime) * 1E-6
//                << " is time and difference of the arm waypoints [Right: " << is_right << "]\n";


    }
    m.trajectory_points.push_back(point);
  }
  return true;
}

Eigen::Isometry3d KDLToEigen(KDL::Frame tf)
{
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion(q.x(), q.y(), q.z(), q.w());
  tf_out.rotate(q);
  return tf_out;
}


bool LCM2ROS::getChestTrajectoryPlan(const drc::robot_plan_t* msg, std::vector<geometry_msgs::Quaternion> &m)
{
  for (int i = 0; i < msg->num_states; i++)  // NB: skipping the first sample as it has time = 0
  {
    // 0. Extract World Pose of body:
    bot_core::robot_state_t this_state = msg->plan[i];
    Eigen::Isometry3d world_to_body;
    world_to_body.setIdentity();
    world_to_body.translation()  << this_state.pose.translation.x, this_state.pose.translation.y, this_state.pose.translation.z;
    world_to_body.rotate(Eigen::Quaterniond(this_state.pose.rotation.w, this_state.pose.rotation.x,
                                                 this_state.pose.rotation.y, this_state.pose.rotation.z));

    // 1. Solve for Forward Kinematics:
    std::map<std::string, double> jointpos_in;
    std::map<std::string, KDL::Frame > cartpos_out;
    for (uint i=0; i< (uint) this_state.num_joints; i++)  // cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(this_state.joint_name[i], this_state.joint_position[i]));

    // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree = true;  // determines absolute transforms to robot origin, otherwise relative transforms between joints.
    kinematics_status = fksolver_->JntToCart(jointpos_in, cartpos_out, flatten_tree);
    if (kinematics_status < 0)
    {
      std::cerr << "Error: could not calculate forward kinematics!" << std::endl;
      return false;
    }

    // 2. Find the world orientation of the chest:
    Eigen::Isometry3d world_to_torso = world_to_body * KDLToEigen(cartpos_out.find(chestLinkName_)->second);
    Eigen::Quaterniond wTt_quat = Eigen::Quaterniond(world_to_torso.rotation());
    geometry_msgs::Quaternion this_chest;
    this_chest.w = wTt_quat.w();
    this_chest.x = wTt_quat.x();
    this_chest.y = wTt_quat.y();
    this_chest.z = wTt_quat.z();
    m.push_back(this_chest);

  }

  return true;
}


void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg)
{
  ROS_ERROR("LCM2ROS got robot plan with %d states", msg->num_states);

  ihmc_msgs::WholeBodyTrajectoryPacketMessage wbt_msg;
  wbt_msg.unique_id = msg->utime;

  // 1. Insert Arm Joints
  std::vector<std::string> l_arm_strings;
  std::vector<std::string> r_arm_strings;
  std::vector<std::string> input_joint_names = msg->plan[0].joint_name;
  if (robotName_.compare("atlas") == 0)
  {
    // Remove MIT/IPAB joint names and use IHMC joint names:
    filterJointNamesToIHMC(input_joint_names);
    l_arm_strings =
    { "l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx", "l_arm_wry2"};
    r_arm_strings =
    { "r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx", "r_arm_wry2"};
  }
  else if (robotName_.compare("valkyrie") == 0)
  {
    l_arm_strings =
    {"leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw",
    "leftWristRoll", "leftWristPitch"};
    r_arm_strings =
    {"rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw",
    "rightWristRoll", "rightWristPitch"};
  }
  ihmc_msgs::ArmJointTrajectoryPacketMessage left_arm_trajectory;
  bool status_left = getSingleArmPlan(msg, l_arm_strings, input_joint_names, false, left_arm_trajectory);
  ihmc_msgs::ArmJointTrajectoryPacketMessage right_arm_trajectory;
  bool status_right = getSingleArmPlan(msg, r_arm_strings, input_joint_names, true, right_arm_trajectory);
  if (!status_left || !status_right)
  {
    ROS_ERROR("LCM2ROS: problem with arm plan, not sending");
    return;
  }
  wbt_msg.left_arm_trajectory = left_arm_trajectory;
  wbt_msg.right_arm_trajectory = right_arm_trajectory;
  wbt_msg.num_joints_per_arm = l_arm_strings.size();

  // 2. Insert Pelvis Pose
  for (int i = 0; i < msg->num_states; i++)  // NB: skipping the first sample as it has time = 0
  {
    bot_core::robot_state_t state = msg->plan[i];

    geometry_msgs::Vector3 pelvis_world_position;
    pelvis_world_position.x = state.pose.translation.x;
    pelvis_world_position.y = state.pose.translation.y;
    pelvis_world_position.z = state.pose.translation.z;
    wbt_msg.pelvis_world_position.push_back(pelvis_world_position);

    geometry_msgs::Quaternion pelvis_world_orientation;
    pelvis_world_orientation.w = state.pose.rotation.w;
    pelvis_world_orientation.x = state.pose.rotation.x;
    pelvis_world_orientation.y = state.pose.rotation.y;
    pelvis_world_orientation.z = state.pose.rotation.z;
    wbt_msg.pelvis_world_orientation.push_back(pelvis_world_orientation);
    wbt_msg.time_at_waypoint.push_back( getPlanTimeAtWaypoint(state.utime) ) ;
    std::cout << i << ": " << getPlanTimeAtWaypoint(state.utime) << " " << (state.utime - msg->plan[i-1].utime) * 1E-6 << " is time and difference of the pelvis waypoints\n";

  }
  wbt_msg.num_waypoints = msg->num_states - 0;  // NB: skipping the first sample as it has time = 0

  // 3. Insert Chest Pose (in work frame)
  std::vector<geometry_msgs::Quaternion> chest_trajectory;
  bool status_chest = getChestTrajectoryPlan(msg, chest_trajectory);
  if (!status_chest)
  {
    ROS_ERROR("LCM2ROS: problem with chest plan, not sending");
    return;
  }
  wbt_msg.chest_world_orientation = chest_trajectory;

  if (outputTrajectoryMode_ == TrajectoryMode::wholeBody)
  {
    //  Temporary fix
    bool send_last = false;
    if(nh_.hasParam("/lcm2ros_ihmc/only_send_last_traj_point"))
      nh_.getParam("/lcm2ros_ihmc/only_send_last_traj_point",send_last);
    if(send_last)
    {
      ROS_WARN_STREAM("Actual trajectory length "<<msg->num_states<<", only last point will be sent");
      wbt_msg.right_arm_trajectory.trajectory_points.resize(1);
      wbt_msg.right_arm_trajectory.trajectory_points[0] = right_arm_trajectory.trajectory_points[right_arm_trajectory.trajectory_points.size()-1];
      wbt_msg.left_arm_trajectory.trajectory_points.resize(1);
      wbt_msg.left_arm_trajectory.trajectory_points[0] = left_arm_trajectory.trajectory_points[left_arm_trajectory.trajectory_points.size()-1];
      bot_core::robot_state_t last_state = msg->plan[msg->num_states-1];
      wbt_msg.pelvis_world_position.resize(1);
      wbt_msg.pelvis_world_position[0].x = last_state.pose.translation.x;
      wbt_msg.pelvis_world_position[0].y = last_state.pose.translation.y;
      wbt_msg.pelvis_world_position[0].z = last_state.pose.translation.z;
      wbt_msg.pelvis_world_orientation.resize(1);
      wbt_msg.pelvis_world_orientation[0].x = last_state.pose.rotation.x;
      wbt_msg.pelvis_world_orientation[0].y = last_state.pose.rotation.y;
      wbt_msg.pelvis_world_orientation[0].z = last_state.pose.rotation.z;
      wbt_msg.pelvis_world_orientation[0].w = last_state.pose.rotation.w;
      wbt_msg.chest_world_orientation.resize(1);
      wbt_msg.chest_world_orientation[0] = chest_trajectory[chest_trajectory.size()-1];
      wbt_msg.time_at_waypoint.resize(1);
      wbt_msg.time_at_waypoint[0] = getPlanTimeAtWaypoint(last_state.utime);
      wbt_msg.num_waypoints = 1;
    }
    whole_body_trajectory_pub_.publish(wbt_msg);
    ROS_ERROR("LCM2ROS sent Whole Body Trajectory");
  }
  else if (outputTrajectoryMode_ == TrajectoryMode::leftArm)
  {
    sendSingleArmPlan(msg, l_arm_strings, input_joint_names, false);
    ROS_ERROR("LCM2ROS sent left arm");
  }
  else if (outputTrajectoryMode_ == TrajectoryMode::rightArm)
  {
    sendSingleArmPlan(msg, r_arm_strings, input_joint_names, true);
    ROS_ERROR("LCM2ROS sent right arm");
  }
  else if (outputTrajectoryMode_ == TrajectoryMode::bothArms)
  {
    sendSingleArmPlan(msg, l_arm_strings, input_joint_names, false);
    ROS_ERROR("LCM2ROS sent left arm");
    sleep(1); 
    ROS_ERROR("LCM2ROS sleep 1 second");
    sendSingleArmPlan(msg, r_arm_strings, input_joint_names, true);
    ROS_ERROR("LCM2ROS sent right arm");
  }

  if (1==0){
    if (robotName_.compare("valkyrie") == 0){

      std::vector<std::string>::iterator it1 = find(input_joint_names.begin(),
          input_joint_names.end(), "lowerNeckPitch");
      int lowerNeckPitchIndex = std::distance(input_joint_names.begin(), it1);
      float lowerNeckPitchAngle = msg->plan[ msg->num_states-1 ].joint_position[lowerNeckPitchIndex];

      std::vector<std::string>::iterator it2 = find(input_joint_names.begin(),
          input_joint_names.end(), "neckYaw");
      int neckYawIndex = std::distance(input_joint_names.begin(), it2);
      float neckYawAngle = msg->plan[ msg->num_states-1 ].joint_position[neckYawIndex];

      Eigen::Quaterniond headOrientation = euler_to_quat(0, lowerNeckPitchAngle, neckYawAngle);
      ihmc_msgs::HeadOrientationPacketMessage mout;
      mout.trajectory_time = 1; // time doesn't seem to be tracked currently.
      mout.orientation.w = headOrientation.w();
      mout.orientation.x = headOrientation.x();
      mout.orientation.y = headOrientation.y();
      mout.orientation.z = headOrientation.z();
      mout.unique_id = msg->utime;
      neck_orientation_pub_.publish(mout);
      ROS_ERROR("LCM2ROS sent desired head orientation");
    }
  }
   
}


void LCM2ROS::ihmcControlModeCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::int64_stamped_t* msg)
{
  outputTrajectoryMode_ = TrajectoryMode( (int) msg->data);
  ROS_ERROR("LCM2ROS Setting Controller TrajectoryMode: %s",
      TrajectoryNames.at( (int) outputTrajectoryMode_).c_str() );
}

int main(int argc, char** argv)
{
  std::string robotName;  // = "valkyrie";  // "atlas"

  if (argc >= 2)
  {
    robotName = argv[1];

    if (!((robotName == "atlas") || robotName == "valkyrie")) {
      ROS_ERROR("Robot name needs to be either atlas or valkyrie");
      return 1;
    }

    ROS_ERROR("Robot Name: %s", argv[1]);
  }
  else
  {
    ROS_ERROR("Need to have an argument: robot name");
    exit(-1);
  }

  ros::init(argc, argv, "lcm2ros_ihmc", ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh, robotName);
  ROS_ERROR("LCM2ROS IHMC Translator Ready [robotName: %s]", robotName.c_str());

  while (0 == lcm->handle())
  {
  }

  return 0;
}
