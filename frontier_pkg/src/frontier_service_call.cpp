#include <ros/ros.h>
#include <frontier_pkg/FrontierMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "service_call_node");
	ros::NodeHandle nh;

	boost::shared_ptr<nav_msgs::OccupancyGrid const> map_msg;
	map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
	geometry_msgs::PoseArray frontier_pts;
	geometry_msgs::PoseStamped goal_frontier;

	ros::ServiceClient frontier_client = nh.serviceClient<frontier_pkg::FrontierMsg>("/frontier_pts");
	frontier_pkg::FrontierMsg srv;
	srv.request.map_data = *map_msg;

	ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/tb3_0/move_base_simple/goal", 1000);
	ros::Publisher frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("/frontier_pose", 1000);

	if (frontier_client.call(srv)){
		ROS_INFO("successfully sent service request");
		frontier_pts = srv.response.optimal_frontier_pts;
		cout <<  frontier_pts.poses.size() << endl;
		goal_frontier.header.stamp = ros::Time::now();
		goal_frontier.header.frame_id = "map";
		goal_frontier.pose = srv.response.optimal_frontier_pts.poses[0];
		goal_frontier.pose.orientation.w = 1.0;
		ros::Duration(4).sleep();
		goal_pub.publish(goal_frontier);
		frontier_pub.publish(goal_frontier);
		

	}

	else {
		ROS_ERROR("failed to call service");
		return 1;
	}


	return 0;
}