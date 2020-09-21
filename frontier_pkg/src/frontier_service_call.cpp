#include <ros/ros.h>
#include <frontier_pkg/FrontierMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_call_node");
	ros::NodeHandle nh;

	boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
	map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");

	ros::ServiceClient frontier_client = nh.serviceClient<frontier_pkg::FrontierMsg>("/frontier_pts");
	frontier_pkg::FrontierMsg srv;
	srv.request.map_data = *map_msg;

	if (frontier_client.call(srv)){
		ROS_INFO("successfully sent service request");
	}

	else {
		ROS_ERROR("failed to call service");
		return 1;
	}


	return 0;
}