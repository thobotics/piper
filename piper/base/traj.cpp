/**
 *  @file   traj.cpp
 *  @brief  trajectory: action client, initialize, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#include <traj.h>


namespace piper {

/* ************************************************************************** */
Traj::Traj(ros::NodeHandle nh)
{
  // ros trajectory joint names
  nh.getParam("robot/arm_joint_names", arm_joint_names);
  traj_.trajectory.joint_names = arm_joint_names;

  // to visualize estimated trajectory
  if (nh.hasParam("robot/est_traj_pub_topic"))
  {
    nh.getParam("robot/est_traj_pub_topic", est_traj_pub_topic_);
    est_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(est_traj_pub_topic_, 1);
  }

  // to visualize planned trajectory
  if (nh.hasParam("robot/plan_traj_pub_topic"))
  {
    nh.getParam("robot/plan_traj_pub_topic", plan_traj_pub_topic_);
    plan_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(plan_traj_pub_topic_, 1);
  }

  // trajectory action client
  if (nh.hasParam("robot/trajectory_control_topic"))
  {
    nh.getParam("robot/trajectory_control_topic", trajectory_control_topic_);
    traj_client_ = new Traj::TrajClient(trajectory_control_topic_, true);
    if (!traj_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for trajectory_control server...");
      if (!traj_client_->waitForServer(ros::Duration(5.0)))
      {
        ROS_ERROR("Cannot find trajectory_control server \'%s\'", trajectory_control_topic_.c_str());
        sigintHandler(0);
      }
    }
  }
  else
    ROS_WARN("No trajectory control topic. Trajectory will not be executed.");

  // TODO: Check parameters
  gplan_client_ = nh.serviceClient<GlobalPlanSrv>("/planner/make_plan");
}

/* ************************************************************************** */
void Traj::initializeTrajectory(gtsam::Values& init_values, Problem& problem)
{
  ROS_INFO("Initializing trajectory.");
  gtsam::Vector conf, avg_vel;
  if (!problem.robot.isMobileBase())
  {
    avg_vel = (problem.goal_conf - problem.start_conf)/problem.total_time;
    for (size_t i=0; i<problem.total_step; i++)
    {
      double ratio = static_cast<double>(i)/static_cast<double>(problem.total_step-1);
      conf = (1.0 - ratio)*problem.start_conf + ratio*problem.goal_conf;
      init_values.insert(gtsam::Symbol('x',i), conf);
      init_values.insert(gtsam::Symbol('v',i), avg_vel);
    }
  }
  else
  {
    gtsam::Pose2 pose;
    if (problem.robot.isDifferentialDrive()){
      avg_vel = (gtsam::Vector(problem.robot.getDOF()-1) << problem.goal_pose.x()-problem.start_pose.x(),
        problem.goal_pose.theta()-problem.start_pose.theta(),
        problem.goal_conf - problem.start_conf).finished()/problem.total_time;
    }else{
      avg_vel = (gtsam::Vector(problem.robot.getDOF()) << problem.goal_pose.x()-problem.start_pose.x(),
        problem.goal_pose.y()-problem.start_pose.y(), problem.goal_pose.theta()-problem.start_pose.theta(),
        problem.goal_conf - problem.start_conf).finished()/problem.total_time;
    }

    for (size_t i=0; i<problem.total_step; i++)
    {
      double ratio = static_cast<double>(i)/static_cast<double>(problem.total_step-1);
      // pose = gtsam::Pose2((1.0 - ratio)*problem.start_pose.x() + ratio*problem.goal_pose.x(),
      //   (1.0 - ratio)*problem.start_pose.y() + ratio*problem.goal_pose.y(),
      //   (1.0 - ratio)*problem.start_pose.theta() + ratio*problem.goal_pose.theta());

      pose = gtsam::Pose2(gpath_init_[i].x(), gpath_init_[i].y(),
        (1.0 - ratio)*problem.start_pose.theta() + ratio*problem.goal_pose.theta());

      conf = (1.0 - ratio)*problem.start_conf + ratio*problem.goal_conf;

      printf("i: %d -- x %f, y %f theta %f \n", i, pose.x(), pose.y(), pose.theta());
      init_values.insert(gtsam::Symbol('x',i), gpmp2::Pose2Vector(pose, conf));
      // init_values.insert(gtsam::Symbol('x',i), gpmp2::Pose2Vector(gpath_init_[i], conf));
      init_values.insert(gtsam::Symbol('v',i), avg_vel);

      // if (i == 0){
      //   problem.pstart = gpmp2::Pose2Vector(gtsam::Pose2(problem.pstart.pose().x(),
      //     problem.pstart.pose().y(), gpath_init_[i].theta()), problem.pstart.configuration());
      // }else if (i == problem.total_step-1){
      //   problem.pgoal = gpmp2::Pose2Vector(gpath_init_[i], conf);
      // }
    }

  }
}

/* ************************************************************************** */
void Traj::executeTrajectory(gtsam::Values& exec_values, Problem& problem, size_t exec_step)
{
  gtsam::Pose2 pose;
  gtsam::Vector conf, vel;
  int DOF = problem.robot.getDOF();
  int DOF_arm = problem.robot.getDOFarm();

  // create ros trajectory
  traj_.trajectory.points.resize(exec_step);
  for (size_t i=0; i<exec_step; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      traj_.trajectory.points[i].positions.resize(DOF_arm);
      traj_.trajectory.points[i].velocities.resize(DOF_arm);      
      conf = exec_values.at<gtsam::Vector>(gtsam::Symbol('x',i));
      vel = exec_values.at<gtsam::Vector>(gtsam::Symbol('v',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (size_t j=0; j<DOF_arm; j++)
      {
        traj_.trajectory.points[i].positions[j] = conf[j];
        traj_.trajectory.points[i].velocities[j] = vel[j];
      }
    }
    else
    {
      int vel_size = problem.robot.isDifferentialDrive() ? DOF -1 : DOF;
      traj_.trajectory.points[i].positions.resize(DOF);
      traj_.trajectory.points[i].velocities.resize(vel_size);
      pose = exec_values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).pose();
      conf = exec_values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).configuration();
      vel = exec_values.at<gtsam::Vector>(gtsam::Symbol('v',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      traj_.trajectory.points[i].positions[0] = pose.x();
      traj_.trajectory.points[i].positions[1] = pose.y();
      traj_.trajectory.points[i].positions[2] = pose.theta();
      for (size_t j=0; j<DOF_arm; j++)
        traj_.trajectory.points[i].positions[j+3] = conf[j];
      for (size_t j=0; j<vel_size; j++)
        traj_.trajectory.points[i].velocities[j] = vel[j];
    }
    traj_.trajectory.points[i].time_from_start = ros::Duration(i*problem.delta_t/(problem.control_inter+1));
  }
  traj_.trajectory.header.stamp = ros::Time::now();
    
  // dispatch ros trajectory
  traj_client_->sendGoal(traj_);
  traj_client_->waitForResult();
}

/* ************************************************************************** */
void Traj::publishEstimatedTrajectory(gtsam::Values& values, Problem& problem, size_t step)
{
  gtsam::Vector conf;
  gtsam::Pose2 pose;
  trajectory_msgs::JointTrajectory est_traj;
  est_traj.points.resize(step+1);
  for (size_t i=0; i<step+1; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      est_traj.points[i].positions.resize(problem.robot.getDOFarm());
      conf = values.at<gtsam::Vector>(gtsam::Symbol('x',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        est_traj.points[i].positions[j] = conf[j];
    }
    else
    {
      est_traj.points[i].positions.resize(problem.robot.getDOF());
      pose = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).pose();
      conf = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).configuration();
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      est_traj.points[i].positions[0] = pose.x();
      est_traj.points[i].positions[1] = pose.y();
      est_traj.points[i].positions[2] = pose.theta();
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        est_traj.points[i].positions[j+3] = conf[j];
    }
  }
  est_traj_pub.publish(est_traj);
}

/* ************************************************************************** */
void Traj::publishPlannedTrajectory(gtsam::Values& values, Problem& problem, size_t step)
{
  gtsam::Vector conf, vel;
  gtsam::Pose2 pose;
  trajectory_msgs::JointTrajectory plan_traj;
  plan_traj.points.resize(problem.total_step-step);
  for (size_t i=step; i<problem.total_step; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      plan_traj.points[i-step].positions.resize(problem.robot.getDOFarm());
      conf = values.at<gtsam::Vector>(gtsam::Symbol('x',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        plan_traj.points[i-step].positions[j] = conf[j];
    }
    else
    {
      plan_traj.points[i-step].positions.resize(problem.robot.getDOF());
      pose = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).pose();
      conf = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).configuration();
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      plan_traj.points[i-step].positions[0] = pose.x();
      plan_traj.points[i-step].positions[1] = pose.y();
      plan_traj.points[i-step].positions[2] = pose.theta();
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        plan_traj.points[i-step].positions[j+3] = conf[j];

      vel = values.at<gtsam::Vector>(gtsam::Symbol('v',i));
      int vel_size = problem.robot.isDifferentialDrive() ? problem.robot.getDOF()-1 : problem.robot.getDOF();
      plan_traj.points[i-step].velocities.resize(problem.robot.getDOF()-1);
      for (size_t j=0; j<vel_size; j++)
        plan_traj.points[i-step].velocities[j] = vel[j];
    }
  }
  plan_traj_pub.publish(plan_traj);
}

/* ************************************************************************** */
void Traj::potentialNavigation(Problem& problem)
{

  geometry_msgs::PoseStamped start, goal;
  start.header.frame_id = "/map";
  start.pose.position.x = problem.start_pose.x();
  start.pose.position.y = problem.start_pose.y();

  goal.header.frame_id = "/map";
  goal.pose.position.x = problem.goal_pose.x();
  goal.pose.position.y = problem.goal_pose.y();

  GlobalPlanSrv gplan_srv;
  gplan_srv.request.start = goal;
  gplan_srv.request.goal = start;
  // gplan_srv.request.start = start;
  // gplan_srv.request.goal = goal;

  if (gplan_client_.call(gplan_srv))
  {
    vector<geometry_msgs::PoseStamped> gpath = gplan_srv.response.path;
    int step = gpath.size() / (problem.total_step - 2);

    gpath_init_.push_back(problem.pstart.pose());
    for (size_t i = problem.total_step - 2; i > 0; i--)
    {
      geometry_msgs::Pose pose = gpath[i*step].pose;
      tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      gpath_init_.push_back(gtsam::Pose2(pose.position.x, pose.position.y, yaw));
      // printf("i %i: x %f, y %f theta %f \n", i*step, pose.position.x, pose.position.y, yaw);
    }
    gpath_init_.push_back(problem.pgoal.pose());
  }
  else
  {
    ROS_ERROR("Failed to call service makeplan");
    // return 1;
  }
}
} // piper namespace
