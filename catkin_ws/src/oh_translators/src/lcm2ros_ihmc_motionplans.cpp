#include "lcm2ros_ihmc.hpp"
#include "lcm2ros_ihmc_conversions.hpp"


void LCM2ROS::handPoseCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg)
{

  ROS_ERROR("LCM2ROS got hand pose on %s" , channel.c_str() );

  ihmc_msgs::SE3TrajectoryPointRosMessage point;
  point.time = 10;

  point.position.x = msg->pos[0];
  point.position.y = msg->pos[1];
  point.position.z = msg->pos[2];
  point.orientation.w = msg->orientation[0];
  point.orientation.x = msg->orientation[1];
  point.orientation.y = msg->orientation[2];
  point.orientation.z = msg->orientation[3];

  point.linear_velocity.x = 0;
  point.linear_velocity.y = 0;
  point.linear_velocity.z = 0;
  point.angular_velocity.x = 0;
  point.angular_velocity.y = 0;
  point.angular_velocity.z = 0;
  point.unique_id = -1;

  std::vector<ihmc_msgs::SE3TrajectoryPointRosMessage> points;
  points.push_back(point);


  ihmc_msgs::HandTrajectoryRosMessage mout;
  mout.base_for_control = 1;//WORLD;
  mout.taskspace_trajectory_points = points;
  mout.execution_mode = 0;//OVERRIDE;
  mout.previous_message_id = -1;
  mout.unique_id = msg->utime;

  if (channel == "HAND_POSE_COMMAND_LEFT")
  {
    mout.robot_side = LEFT;
  }
  else if (channel == "HAND_POSE_COMMAND_RIGHT")
  {
    mout.robot_side = RIGHT;
  }

  hand_pose_command_pub_.publish(mout);
}


void LCM2ROS::ihmcControlModeCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::int64_stamped_t* msg)
{
  outputTrajectoryMode_ = TrajectoryMode( (int) msg->data);
  ROS_ERROR("LCM2ROS Setting Controller TrajectoryMode: %s",
      getTrajectoryName( (int) outputTrajectoryMode_) );

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
  ihmc_msgs::ArmTrajectoryRosMessage m;
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
                               ihmc_msgs::ArmTrajectoryRosMessage &m)
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


  for (int j = 0; j < arm_indices.size(); j++)
  {

    ihmc_msgs::OneDoFJointTrajectoryRosMessage joint_trajectory_message;

    std::vector<ihmc_msgs::TrajectoryPoint1DRosMessage> points;
    for (int i = 0; i < msg->num_states; i++)  // NB: skipping the first sample as it has time = 0
    {
      bot_core::robot_state_t state = msg->plan[i];
      ihmc_msgs::TrajectoryPoint1DRosMessage point;
      int i1 = (i > 0) ? (i - 1) : 0;
      int i2 = i;
      int i3 = (i < msg->num_states - 1) ? (i + 1) : (msg->num_states - 1);

      point.position = state.joint_position[arm_indices[j]];
      double dt1 = (msg->plan[i2].utime - msg->plan[i1].utime) * 1e-6;
      double dt2 = (msg->plan[i3].utime - msg->plan[i2].utime) * 1e-6;
      double dq1 = msg->plan[i2].joint_position[arm_indices[j]] - msg->plan[i1].joint_position[arm_indices[j]];
      double dq2 = msg->plan[i3].joint_position[arm_indices[j]] - msg->plan[i2].joint_position[arm_indices[j]];
      point.velocity = (dt1 * dt2 != 0) ? (dq1 / dt1 * 0.5 + dq2 / dt2 * 0.5) : 0.0;
      // point.accelerations.push_back( 0  );
      // point.effort.push_back( state.joint_effort[ arm_indices[j] ] );

      point.time = getPlanTimeAtWaypoint( state.utime );

      points.push_back(point);
    }
    joint_trajectory_message.trajectory_points = points;
    m.joint_trajectory_messages.push_back( joint_trajectory_message );
  }

  m.execution_mode = 0;//OVERRIDE;
  m.previous_message_id = -1; // not used for override
  m.unique_id = msg->utime; // this is now required for execution

/*
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
*/

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


KDL::Twist LCM2ROS::Interpolate(double t1, double t2, double t3, const KDL::Frame& T1, const KDL::Frame& T2, const KDL::Frame& T3)
{
    KDL::Twist ret;
    double dt1 = t2-t1;
    double dt2 = t3-t2;

    if(dt1*dt2>0.0)
    {
        ret.vel = (T2.p-T1.p)/dt1*0.5 + (T3.p-T2.p)/dt2*0.5;
        KDL::Rotation dq1KDL=T1.M.Inverse()*T2.M;
        KDL::Rotation dq2KDL=T2.M.Inverse()*T3.M;
        Eigen::Quaterniond dq1;
        Eigen::Quaterniond dq2;
        dq1KDL.GetQuaternion(dq1.x(),dq1.y(),dq1.z(),dq1.w());
        dq2KDL.GetQuaternion(dq2.x(),dq2.y(),dq2.z(),dq2.w());
        Eigen::Quaterniond dq = Eigen::Quaterniond::Identity().slerp(1.0/dt1*0.5,dq1)*Eigen::Quaterniond::Identity().slerp(1.0/dt2*0.5,dq2);
        //Eigen::Quaterniond dq = dq1.slerp(dt2/(dt1+dt2),dq2);
        Eigen::AngleAxisd tmp(dq);
        Eigen::Vector3d velvec=tmp.axis()*tmp.angle();
        ret.rot=KDL::Vector(velvec(0),velvec(1),velvec(2));
    }

    return ret;
}

bool LCM2ROS::getChestTrajectoryPlan(const drc::robot_plan_t* msg, ihmc_msgs::ChestTrajectoryRosMessage& m)
{
    std::vector<KDL::Frame> world_to_chest;
    for (int i = 0; i < msg->num_states; i++)
    {

        bot_core::robot_state_t this_state = msg->plan[i];

        // 0. Extract World Pose of body:
        KDL::Frame world_to_body(KDL::Rotation::Quaternion(this_state.pose.rotation.x,this_state.pose.rotation.y, this_state.pose.rotation.z,this_state.pose.rotation.w),
                                 KDL::Vector(this_state.pose.translation.x, this_state.pose.translation.y, this_state.pose.translation.z));

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
        world_to_chest.push_back(world_to_body*cartpos_out.at(chestLinkName_));
    }

    for (int i = 0; i < msg->num_states; i++)
    {
        bot_core::robot_state_t this_state = msg->plan[i];
        ihmc_msgs::SO3TrajectoryPointRosMessage point;
        point.time = getPlanTimeAtWaypoint( this_state.utime );

        // Copy orientation
        world_to_chest[i].M.GetQuaternion(point.orientation.x,point.orientation.y,point.orientation.z,point.orientation.w);

        // Compute velocity
        if(i==0||i==msg->num_states-1)
        {
            point.angular_velocity.x = 0;
            point.angular_velocity.y = 0;
            point.angular_velocity.z = 0;
        }
        else
        {
            KDL::Twist vel = Interpolate(msg->plan[i-1].utime*1e-6,msg->plan[i].utime*1e-6,msg->plan[i+1].utime*1e-6,
                    world_to_chest[i-1],world_to_chest[i],world_to_chest[i+1]);
            point.angular_velocity.x = vel.rot.x();
            point.angular_velocity.y = vel.rot.y();
            point.angular_velocity.z = vel.rot.z();
        }

        point.unique_id = -1;
        m.taskspace_trajectory_points.push_back(point);
    }

    return true;
}


void LCM2ROS::getPelvisTrajectoryPlan(const drc::robot_plan_t* msg, ihmc_msgs::PelvisTrajectoryRosMessage& m)
{
    // Collect frames
    std::vector<KDL::Frame> pelvis;

    for (int i = 0; i < msg->num_states; i++)
    {
        bot_core::robot_state_t state = msg->plan[i];
        pelvis.push_back(KDL::Frame(KDL::Rotation::Quaternion(state.pose.rotation.x,state.pose.rotation.y,state.pose.rotation.z,state.pose.rotation.w),
                                    KDL::Vector(state.pose.translation.x,state.pose.translation.y,state.pose.translation.z)));
    }

    for (int i = 0; i < msg->num_states; i++)
    {
        bot_core::robot_state_t this_state = msg->plan[i];
        ihmc_msgs::SE3TrajectoryPointRosMessage point;
        point.time = getPlanTimeAtWaypoint( this_state.utime );

        // Copy position and orientation
        point.position.x = pelvis[i].p.x();
        point.position.y = pelvis[i].p.y();
        point.position.z = pelvis[i].p.z();
        pelvis[i].M.GetQuaternion(point.orientation.x,point.orientation.y,point.orientation.z,point.orientation.w);
        // Compute velocity
        if(i==0||i==msg->num_states-1)
        {
            point.linear_velocity.x = 0;
            point.linear_velocity.y = 0;
            point.linear_velocity.z = 0;
            point.angular_velocity.x = 0;
            point.angular_velocity.y = 0;
            point.angular_velocity.z = 0;
        }
        else
        {
            KDL::Twist vel = Interpolate(msg->plan[i-1].utime*1e-6,msg->plan[i].utime*1e-6,msg->plan[i+1].utime*1e-6,
                    pelvis[i-1],pelvis[i],pelvis[i+1]);
            point.linear_velocity.x = vel.vel.x();
            point.linear_velocity.y = vel.vel.y();
            point.linear_velocity.z = vel.vel.z();
            point.angular_velocity.x = vel.rot.x();
            point.angular_velocity.y = vel.rot.y();
            point.angular_velocity.z = vel.rot.z();
        }

        point.unique_id = -1;
        m.taskspace_trajectory_points.push_back(point);
    }
}

void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg)
{
  ROS_ERROR("LCM2ROS got robot plan with %d states", msg->num_states);

  ihmc_msgs::WholeBodyTrajectoryRosMessage wbt_msg;
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
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_trajectory;
  bool status_left = getSingleArmPlan(msg, l_arm_strings, input_joint_names, false, left_arm_trajectory);
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_trajectory;
  bool status_right = getSingleArmPlan(msg, r_arm_strings, input_joint_names, true, right_arm_trajectory);
  if (!status_left || !status_right)
  {
    ROS_ERROR("LCM2ROS: problem with arm plan, not sending");
    return;
  }
  wbt_msg.left_arm_trajectory_message = left_arm_trajectory;
  wbt_msg.right_arm_trajectory_message = right_arm_trajectory;
  //wbt_msg.num_joints_per_arm = l_arm_strings.size();


  // 2. Insert Pelvis Pose
  getPelvisTrajectoryPlan(msg,wbt_msg.pelvis_trajectory_message);

  // 3. Insert Chest Pose (in work frame)

  bool status_chest = getChestTrajectoryPlan(msg,  wbt_msg.chest_trajectory_message);
  if (!status_chest)
  {
    ROS_ERROR("LCM2ROS: problem with chest plan, not sending");
    return;
  }



  // Various bits and bobs required for the WBT to do <something> at present
  // the below are all hacks but are needed  until some bugs from IHMC are resolved
  wbt_msg.left_hand_trajectory_message.unique_id = 0;
  wbt_msg.right_hand_trajectory_message.unique_id = 0;
  wbt_msg.chest_trajectory_message.unique_id = 1;
  wbt_msg.pelvis_trajectory_message.unique_id = 1;
  wbt_msg.left_foot_trajectory_message.unique_id = 0;
  wbt_msg.right_foot_trajectory_message.unique_id = 0;

  wbt_msg.left_hand_trajectory_message.execution_mode = 0;
  wbt_msg.right_hand_trajectory_message.execution_mode = 0;
  wbt_msg.chest_trajectory_message.execution_mode = 0;
  wbt_msg.pelvis_trajectory_message.execution_mode = 0;
  wbt_msg.left_foot_trajectory_message.execution_mode = 0;
  wbt_msg.right_foot_trajectory_message.execution_mode = 0;

  wbt_msg.right_hand_trajectory_message.robot_side = 1;
  wbt_msg.right_foot_trajectory_message.robot_side = 1;

  // end hacks


  if (outputTrajectoryMode_ == TrajectoryMode::wholeBody)
  {
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
  
}
