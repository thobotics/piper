#include <diff_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>


namespace piper {

/* ************************************************************************** */
DiffInterface::DiffInterface(ros::NodeHandle nh)
{
  // first load problem and setup trajectory client
  problem_ = Problem(nh);
  traj_ = Traj(nh);

  // robot state subscriber
  if (nh.hasParam("robot/arm_state_topic"))
  {
    nh.getParam("robot/arm_state_topic", arm_state_topic_);
    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &DiffInterface::armStateCallback, this);
    arm_pos_ = gtsam::Vector::Zero(problem_.robot.getDOFarm());
    arm_pos_time_ = ros::Time::now();
  }
  // robot state subscriber
  if (problem_.robot.isMobileBase() && nh.hasParam("robot/base_state_topic"))
  {
    nh.getParam("robot/base_state_topic", base_state_topic_);
    potential_topic_ = "/planner/planner/potential";
    base_state_sub_ = nh.subscribe(base_state_topic_, 1, &DiffInterface::baseStateCallback, this);
    navfn_cost_sub_ = nh.subscribe(potential_topic_, 1, &DiffInterface::navCostCallback, this);
    pot_grad_pub_ = nh.advertise<geometry_msgs::PoseArray>("/pot_grad_topic", 1);
    base_pos_ = gtsam::Pose2();
    base_pos_time_ = ros::Time::now();
    nav_cost_time_ = ros::Time::now();
  }
  ros::Duration(1.0).sleep();

  // get start from measurement if not passed as param
  if (problem_.robot.isMobileBase())
  {
    if (!nh.hasParam("start_pose"))
      problem_.start_pose = base_pos_;
    problem_.pstart = gpmp2::Pose2Vector(problem_.start_pose, problem_.start_conf);
  }

  // call global planner
  traj_.potentialNavigation(problem_);

  // initialize trajectory
  traj_.initializeTrajectory(init_values_, problem_);

  // Publish init plan
  if (traj_.plan_traj_pub)
    traj_.publishPlannedTrajectory(init_values_, problem_, 0);

  // solve for initial plan with batch gpmp2
  ROS_INFO("Optimizing...");
  int DOF = problem_.robot.getDOF();
  batch_values_ = piper::BatchTrajOptimizePose2MobileDiff(problem_.robot.marm, problem_.sdf, potential_arr_,
     problem_.pstart, gtsam::Vector::Zero(DOF-1), problem_.pgoal, gtsam::Vector::Zero(DOF-1), init_values_, problem_.opt_setting);
  ROS_INFO("Batch optimization complete.");

  // publish trajectory for visualization or other use
  if (traj_.plan_traj_pub)
    traj_.publishPlannedTrajectory(batch_values_, problem_, 0);

  ros::Duration(3.0).sleep();

  // set up incremental inference
  ROS_INFO("Initializing incremental inference...");
  marm_inc_inf_ = piper::ISAM2TrajOptimizerPose2MobileDiff(problem_.robot.marm, problem_.sdf, potential_arr_, problem_.opt_setting);
  marm_inc_inf_.initFactorGraph(problem_.pstart, gtsam::Vector::Zero(DOF-1), problem_.pgoal, gtsam::Vector::Zero(DOF-1));
  marm_inc_inf_.initValues(batch_values_);
  marm_inc_inf_.update();
  inc_inf_values_ = marm_inc_inf_.values();

  // View difference between Batch solution and ISAM Init factor graph
  traj_.publishPlannedTrajectory(inc_inf_values_, problem_, 0);
  ROS_INFO("Online incremental inference ready.");
}

/* ************************************************************************** */
void DiffInterface::execute()
{
  size_t exec_step;
  double coll_cost;
  gtsam::Matrix sensor_model;
  int DOF_arm = problem_.robot.getDOFarm();
  int DOF = problem_.robot.getDOF();
  gtsam::Vector conf;
  gtsam::Pose2 pose;

  // sensor model for measurements
  sensor_model = (gtsam::Matrix(DOF, DOF) <<
    problem_.robot.sensor_base_sigma*gtsam::Matrix::Identity(3, 3), gtsam::Matrix::Zero(3, DOF_arm),
    gtsam::Matrix::Zero(DOF_arm, 3), problem_.robot.sensor_arm_sigma*gtsam::Matrix::Identity(DOF_arm, DOF_arm)).finished();

  // solve and execute MPEPC problem
  ROS_INFO("Executing MPEPC online...");
  for (size_t step=0; step<problem_.total_step-1; step++)
  {
    // interpolate updated solution to a desired resolution for control until next step and check for collision
    exec_values_ = piper::interpolatePose2DiffArmTraj(inc_inf_values_, problem_.opt_setting.Qc_model,
      problem_.delta_t, problem_.control_inter, step, step+1);
    coll_cost = gpmp2::CollisionCostPose2MobileArm(problem_.robot.marm, problem_.sdf, exec_values_, problem_.opt_setting);

    if (coll_cost != 0)
    {
      ROS_FATAL_STREAM("At step = "<<step<<", plan is not collision free! Collision cost = "<<coll_cost);
      sigintHandler(0);
    }

    // execute trajectory
    exec_step = problem_.control_inter + 2;
    traj_.executeTrajectory(exec_values_, problem_, exec_step);

    // get current state and use if it was measured recently then
    // update factor graph to perform incremental inference
    if (((ros::Time::now() - arm_pos_time_).toSec() < 5) && ((ros::Time::now() - base_pos_time_).toSec() < 5))
    {
      pose = base_pos_;
      conf = arm_pos_;
      if (problem_.robot.isThetaNeg())
        problem_.robot.negateTheta(conf);
      // update
      marm_inc_inf_.addPoseEstimate(step+1, gpmp2::Pose2Vector(pose, conf), sensor_model);
      marm_inc_inf_.update();
      inc_inf_values_ = marm_inc_inf_.values();
    }

    // publish trajectory for visualization or other use
    if (traj_.est_traj_pub)
      traj_.publishEstimatedTrajectory(inc_inf_values_, problem_, step+1);
    if (traj_.plan_traj_pub)
      traj_.publishPlannedTrajectory(inc_inf_values_, problem_, step+1);
  }
}

/* ************************************************************************** */
void DiffInterface::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  size_t index;
  for (size_t i=0; i<problem_.robot.getDOFarm(); i++)
  {
    index = std::distance(msg->name.begin(), find(msg->name.begin(), msg->name.end(),
      traj_.arm_joint_names[i]));
    arm_pos_[i] = msg->position[index];
  }
  arm_pos_time_ = ros::Time::now();
}

/* ************************************************************************** */
void DiffInterface::baseStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  base_pos_ = gtsam::Pose2(msg->position.x, msg->position.y, gtsam::Rot3::Quaternion(msg->orientation.w,
    msg->orientation.x, msg->orientation.y, msg->orientation.z).yaw());
  base_pos_time_ = ros::Time::now();
}

/* ************************************************************************** */
void DiffInterface::navCostCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
  double score_scale = 0.001;
  double scale_res = 100*grid->info.resolution;

  // Parse data
  gtsam::Matrix data2d(grid->info.height, grid->info.width);

  gtsam::Vector row = gtsam::Vector::Zero(grid->info.width);
  for(int i=0; i<grid->info.height; i++){
    for(int j=0; j<grid->info.width; j++){
      double tmp_score = grid->data[i*grid->info.height+j];
      row[j] = tmp_score == -1 ? 100 : tmp_score;
      row[j] *= score_scale;
    }
    data2d.row(i) = row;
  }

  // Load metadata
  gtsam::Point2 origin(grid->info.origin.position.x,
    grid->info.origin.position.y);
  double cell_size = grid->info.resolution;

  // Save SDF
  potential_arr_ = piper::NavPotential(origin, cell_size, scale_res, data2d);

  ROS_INFO("[NavPotential] Width %d, Height %d", grid->info.width, grid->info.height);
  ROS_INFO("[NavPotential] Origin %f %f, Resolution %f", origin[0], origin[1], cell_size);

  // Debugging
  geometry_msgs::PoseArray grad_poses;
  gtsam::Vector2 gradient;
  double potential;
  double max_score = score_scale*(100.0*scale_res*scale_res - 1);
  grad_poses.header.frame_id = "/map";
  grad_poses.header.stamp = ros::Time::now();
  for (int y=1; y<(int)grid->info.height*cell_size-1; y++){
    for (int x=1; x<(int)grid->info.width*cell_size-1; x++){
      potential = potential_arr_.getSignedDistance(gtsam::Point2(x,y), gradient);

      if(potential < max_score){
        geometry_msgs::Pose grad_pose;
        grad_pose.position.x = x;
        grad_pose.position.y = y;

        // Create this quaternion from roll/pitch/yaw (in radians)
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY( 0, 0, atan2(-gradient[1], -gradient[0]) );
        grad_pose.orientation.z = myQuaternion.z();
        grad_pose.orientation.w = myQuaternion.w();

        grad_poses.poses.push_back(grad_pose);

        ROS_DEBUG("[NavPotential] x %d, y %d: potential %f \t gradient %f, %f",
            x, y, potential, gradient[0], gradient[1]);
      }
    }
  }
  pot_grad_pub_.publish(grad_poses);

  nav_cost_time_ = ros::Time::now();
}

} // piper namespace


/* ************************************************************************** */
/* main callback */
void mainCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ros::NodeHandle nh("piper");
  piper::DiffInterface diff(nh);
  // diff.execute();
  ROS_INFO("Done.");
  ros::shutdown();
}

/* ************************************************************************** */
/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_interface");
  signal(SIGINT, piper::sigintHandler);
  ros::MultiThreadedSpinner spinner(0);

  ros::NodeHandle n;
  ros::Publisher main_pub = n.advertise<std_msgs::Bool>("/piper/run_main", 1);
  ros::Subscriber main_sub = n.subscribe("/piper/run_main", 1, mainCallback);
  main_pub.publish(std_msgs::Bool());

  spinner.spin();
}
