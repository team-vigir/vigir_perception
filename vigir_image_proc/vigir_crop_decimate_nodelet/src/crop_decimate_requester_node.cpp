#include "ros/ros.h"
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vigir_crop_decimate_requester_nodelet");

  // Shared parameters to be propagated to nodelet private namespaces
  nodelet::Loader manager(true); // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  manager.load(ros::this_node::getName(), "vigir_crop_decimate_request_nodelet/CropDecimateRequestNodelet", remappings, my_argv);

  ros::spin();
  return 0;
}
