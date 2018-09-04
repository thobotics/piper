#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <ros_sdf.h>
#include <std_msgs/Bool.h>
#include <piper/FieldGrid.h>

#include <gpmp2/obstacle/SignedDistanceField.h>

namespace piper {

/* ************************************************************************** */
RosSDF::RosSDF(ros::NodeHandle nh)
{

}

}

ros::Publisher fieldgrid_ack_pub_;
ros::Subscriber fieldgrid_sub_;

void field_grid_cb(const piper::FieldGridConstPtr &grid){
  // piper::RosSDF sdf;

  // Parse data
  gtsam::Matrix data2d(grid->info.width, grid->info.height);

  gtsam::Vector row = gtsam::Vector::Zero(grid->info.height);
  for(int i=0; i<grid->info.width; i++){
    for(int j=0; j<grid->info.height; j++){
      row[j] = grid->data[i*grid->info.width+j];
    }
    data2d.row(i) = row;
  }

  // Debuging
  // gtsam::print(data2d, "Matrix", std::cout);

  std::vector<gtsam::Matrix> data;
  for(int i=0; i<(0.5/grid->info.resolution); i++)
    data.push_back(data2d);

  // Load metadata
  gtsam::Point3 origin(grid->info.origin.position.x, grid->info.origin.position.y,
                        grid->info.origin.position.z);
  double cell_size = grid->info.resolution;

  // Save SDF
  gpmp2::SignedDistanceField sdf(origin, cell_size, data);
  sdf.print("");
  sdf.saveSDF("./src/piper/sdf/SocialContexts.bin");

  // Debuging
  // gpmp2::SignedDistanceField sdf2(origin, cell_size, grid->info.width, grid->info.height, 1);
  // sdf2.loadSDF("./src/piper/sdf/SocialContexts.bin");
  // gtsam::print(sdf2.raw_data()[0], "Matrix 2", std::cout);

  // Send ACK
  std_msgs::Bool ack;
  ack.data = true;
  fieldgrid_ack_pub_.publish(ack);
}

/* ************************************************************************** */
/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_sdf");
  // signal(SIGINT, piper::sigintHandler);
  // ros::MultiThreadedSpinner spinner(0);

  ros::NodeHandle n;

  printf("ROS_SDF started..\n");
  fieldgrid_ack_pub_  = n.advertise<std_msgs::Bool>("/ack_map_grid", 1);
  fieldgrid_sub_ = n.subscribe("/map_grid", 1, field_grid_cb);

  // spinner.spin();

  ros::Rate r(0.5);
  while (ros::ok())
  {
    ros::spinOnce();                   // Handle ROS events
    r.sleep();
  }
}
