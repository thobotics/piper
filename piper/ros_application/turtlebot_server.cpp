#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

bool is_diff_drive_;
ros::Publisher base_pose_pub_;
ros::Publisher arm_pub_;
ros::Subscriber odom_sub_;
tf::TransformListener *listener_;

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
  ros::Publisher cmd_pub_;

public:

  TrajAction(ros::NodeHandle nh, std::string name) :
    as_(nh, name, boost::bind(&TrajAction::executeCB, this, _1), false),
    action_name_(name)
  {
    nh_ = nh;
    as_.start();
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
  }

  ~TrajAction(void)
  {
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    double total_time = 20.0;
    double total_step = 40;
    double control_inter = 10;

    double step_time = total_time / total_step;
    double iter_time = step_time / control_inter;

    // helper variables
    ros::Rate r(1/iter_time);
    bool success = true;
    geometry_msgs::Twist twist;

    // publish info to the console for the user
    ROS_INFO("%s: Executing", action_name_.c_str());

    // start executing the action
    for (int i=0; i < goal->trajectory.points.size(); i++){
      trajectory_msgs::JointTrajectoryPoint p = goal->trajectory.points[i];

      if (is_diff_drive_){
          twist.linear.x = p.velocities[0];
          twist.angular.z = p.velocities[1];
      }else{
          twist.linear.x = p.velocities[0];
          twist.linear.y = p.velocities[1];
          twist.angular.z = p.velocities[2];
      }

      cmd_pub_.publish(twist);

      r.sleep();
    }

    if(success)
    {
      result_.result.error_code = result_.result.SUCCESSFUL;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_.result);
    }
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

  // Create join message
  sensor_msgs::JointState j_state;
  j_state.name     = {"right_shoulder_pan_joint", "right_shoulder_lift_joint",
        "right_elbow_joint", "right_wrist_1_joint",
        "right_wrist_2_joint", "right_wrist_3_joint"};
  j_state.position = {0.0,0.0,0.0,0.0,0.0,0.0};
  j_state.velocity = {0.0,0.0,0.0,0.0,0.0,0.0};

  base_pose_pub_.publish(map_pose.pose);
  arm_pub_.publish(j_state);
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
