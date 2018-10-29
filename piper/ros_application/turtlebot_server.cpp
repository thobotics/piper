#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <control_law.h>

bool is_diff_drive_;
ros::Publisher base_pose_pub_;
ros::Publisher arm_pub_;
ros::Subscriber odom_sub_;
tf::TransformListener *listener_;
geometry_msgs::Pose current_pose_;

// Mutex
boost::mutex current_pose_mutex_;

/* ************************************************************************** */
// Template from http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
class TrajAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryActionFeedback feedback_;
  control_msgs::FollowJointTrajectoryActionResult result_;
  ros::Publisher l_plan_pub_, cmd_pub_;

  ControlLawSettings settings_;
  ControlLaw * cl_;

public:

  TrajAction(ros::NodeHandle nh, std::string name) :
    as_(nh, name, boost::bind(&TrajAction::executeCB, this, _1), false),
    action_name_(name)
  {
    nh_ = nh;
    as_.start();
    l_plan_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/turtlebot_local_plan", 1);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);

    // Initialize Motion Model
		settings_.m_K1 = 0.5;//K_1;
		settings_.m_K2 = 1.0;//K_2;
		// settings_.m_BETA = BETA;
		// settings_.m_LAMBDA = LAMBDA;
		// settings_.m_R_THRESH = R_THRESH;
		settings_.m_V_MAX = 1.5;
		// settings_.m_V_MIN = W_TURN;//V_MIN;
		cl_ = new ControlLaw(&settings_);
  }

  ~TrajAction(void)
  {
  }

  void publish_feedback(){
      // Create join message
      sensor_msgs::JointState j_state;
      j_state.name     = {"right_shoulder_pan_joint", "right_shoulder_lift_joint",
            "right_elbow_joint", "right_wrist_1_joint",
            "right_wrist_2_joint", "right_wrist_3_joint"};
      j_state.position = {0.0,0.0,0.0,0.0,0.0,0.0};
      j_state.velocity = {0.0,0.0,0.0,0.0,0.0,0.0};

      base_pose_pub_.publish(current_pose_);
      arm_pub_.publish(j_state);
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    double total_time = 20.0;
    double total_step = 40;
    double control_inter = 10;

    double step_time = total_time / total_step;
    double iter_time = step_time / control_inter;

    // helper variables
    // ros::Rate r(1/iter_time);
    geometry_msgs::Twist twist;
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("%s: Executing...", action_name_.c_str());

    double TIME_HORIZON = 0.5;
    double DELTA_SIM_TIME = 0.05;
    ros::Rate r(1/DELTA_SIM_TIME);

    int goal_idx = goal->trajectory.points.size() - 1;
    geometry_msgs::Pose current_position, current_goal;

    current_position.position.x = goal->trajectory.points[0].positions[0];
    current_position.position.y = goal->trajectory.points[0].positions[1];
    current_position.orientation = tf::createQuaternionMsgFromYaw(
      goal->trajectory.points[0].positions[2]);

    current_goal.position.x = goal->trajectory.points[goal_idx].positions[0];
    current_goal.position.y = goal->trajectory.points[goal_idx].positions[1];
    current_goal.orientation = tf::createQuaternionMsgFromYaw(
      goal->trajectory.points[goal_idx].positions[2]);

    geometry_msgs::PoseArray viz_plan;
  	viz_plan.header.stamp = ros::Time::now();
  	viz_plan.header.frame_id = "/map";
  	viz_plan.poses.resize(1);

    std::vector<geometry_msgs::Twist> cmd_vels;

    geometry_msgs::Pose sim_pose = current_pose_;
    geometry_msgs::Twist sim_cmd_vel;
	  double current_yaw = tf::getYaw(sim_pose.orientation);
    double sim_clock = 0.0;

    ROS_INFO("[Exec] start %.2f %.2f %.2f, goal %.2f %.2f %.2f",
        current_position.position.x, current_position.position.y, goal->trajectory.points[0].positions[2],
        current_goal.position.x, current_goal.position.y, goal->trajectory.points[goal_idx].positions[2]
      );

    while (sim_clock < TIME_HORIZON)
  	{
  	  sim_cmd_vel = cl_->get_velocity_command(
        sim_pose, current_goal,
        settings_.m_K1, settings_.m_K2, settings_.m_V_MAX
      );

  	  // Update pose
  	  current_yaw = current_yaw + (sim_cmd_vel.angular.z * DELTA_SIM_TIME);
  	  sim_pose.position.x = sim_pose.position.x + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * cos(current_yaw));
  	  sim_pose.position.y = sim_pose.position.y + (sim_cmd_vel.linear.x * DELTA_SIM_TIME * sin(current_yaw));
  	  sim_pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw);
  	  viz_plan.poses.push_back(sim_pose);
      cmd_vels.push_back(sim_cmd_vel);

  	  sim_clock = sim_clock + DELTA_SIM_TIME;

      ROS_INFO("[Exec...] sim_pose %.2f %.2f %.2f, vel %.2f %.2f",
          sim_pose.position.x, sim_pose.position.y, current_yaw,
          sim_cmd_vel.linear.x, sim_cmd_vel.angular.z
        );
  	}

    l_plan_pub_.publish(viz_plan);

    for (int i=0; i < cmd_vels.size(); i++){
        cmd_pub_.publish(cmd_vels[i]);
        r.sleep();
    }

    // start executing the action
    // for (int i=0; i < goal->trajectory.points.size(); i++){
    //   trajectory_msgs::JointTrajectoryPoint p = goal->trajectory.points[i];
    //
    //   ROS_INFO("[Exec] pose %.2f %.2f %.2f, vel %.2f %.2f",
    //     p.positions[0], p.positions[1], p.positions[2],
    //     p.velocities[0], p.velocities[1]
    //   );
    //
    //   if (is_diff_drive_){
    //       twist.linear.x = p.velocities[0];
    //       twist.angular.z = p.velocities[1];
    //   }else{
    //       twist.linear.x = p.velocities[0];
    //       twist.linear.y = p.velocities[1];
    //       twist.angular.z = p.velocities[2];
    //   }
    //
    //   cmd_pub_.publish(twist);
    //
    //   r.sleep();
    // }

    if(success)
    {
      result_.result.error_code = result_.result.SUCCESSFUL;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_.result);
    }

    // publish current state to client
    publish_feedback();

  }

};


/* ************************************************************************** */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  try{
      listener_->waitForTransform("/map", "/odom",
                               ros::Time(0), ros::Duration(4.0));
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    // continue;
  }

  // Generate odom pose message and transform to map pose
  geometry_msgs::PoseStamped odom;
  odom.header.frame_id = "odom";
  odom.header.stamp    = ros::Time(0);
  odom.pose            = msg->pose.pose;

  geometry_msgs::PoseStamped map_pose;
  listener_->transformPose("map", odom, map_pose);

  current_pose_ = map_pose.pose;
}

/* ************************************************************************** */
/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_server");
  ros::NodeHandle n;
  ROS_INFO("turtlebot_server started..\n");

  n.getParam("/turtlebot_server/diff_drive", is_diff_drive_);
  base_pose_pub_ = n.advertise<geometry_msgs::Pose>("/base_state", 10);
  arm_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
  odom_sub_ = n.subscribe("/odom", 1, odomCallback);
  listener_ = new tf::TransformListener(); // Listener must be init after ros::init()

  TrajAction server(n, "/vector/full_body_controller/trajectory");

  ros::spin();
}
