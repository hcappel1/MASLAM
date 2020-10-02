#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <frontier_pkg/ChoiceMsg.h>
#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <list>
#include <algorithm>

using namespace std;

class FrontierPt{

public:

	FrontierPt(){

	}

	geometry_msgs::Pose pose;
	vector< shared_ptr<FrontierPt> > neighbors;
	bool valid;
	string key; 
};

class ChooseFrontier{

private:
	ros::NodeHandle nh;
	ros::Publisher frontier_pub;
	ros::Publisher neighbors_pub;

	geometry_msgs::PoseArray optimal_frontier_pts;
	vector< shared_ptr<FrontierPt> > frontier_queue;
	vector< shared_ptr<FrontierPt> > valid_frontier_queue;
	shared_ptr<FrontierPt> candidate_pt;
	vector<vector< shared_ptr<FrontierPt> >> source_paths;
	vector< shared_ptr<FrontierPt> > definite_frontier_queue;
	vector< shared_ptr<FrontierPt> > valid_path_list;


public:
	ChooseFrontier(){
		ROS_INFO("choose frontier object created");
		frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("chosen_frontier_pt", 1000);
		neighbors_pub = nh.advertise<geometry_msgs::PoseArray>("neighbor_frontier_pts", 1000);
	}

	bool ServiceCallback(frontier_pkg::ChoiceMsg::Request &req,
						frontier_pkg::ChoiceMsg::Response &res)
	{
		optimal_frontier_pts = req.optimal_frontier_pts;
		CreateFrontierQueue();
		GetNeighbors();
		NeighborsTest();

		res.success = true;
	}

	void CreateFrontierQueue(){

		for (int i = 0; i < optimal_frontier_pts.poses.size(); i++){

			shared_ptr<FrontierPt> new_frontier_pt = shared_ptr<FrontierPt>( new FrontierPt );
			new_frontier_pt->pose = optimal_frontier_pts.poses[i];
			new_frontier_pt->valid = false;
			new_frontier_pt->key = (to_string(optimal_frontier_pts.poses[i].position.x) + "-" + to_string(optimal_frontier_pts.poses[i].position.y));
			frontier_queue.push_back(new_frontier_pt);
		}
	}

	void GetNeighbors(){

		for (int i = 0; i < frontier_queue.size(); i++){
			for (int j = 0; j < frontier_queue.size(); j++){

				if (frontier_queue[i]->key != frontier_queue[j]->key){

					double distance = sqrt(pow(frontier_queue[i]->pose.position.x - frontier_queue[j]->pose.position.x,2) + pow(frontier_queue[i]->pose.position.y - frontier_queue[j]->pose.position.y,2));
					if (distance < 3.0){
						frontier_queue[i]->neighbors.push_back(frontier_queue[j]);
					}
				}
			}
		}
	}

	void NeighborsTest(){

		geometry_msgs::PoseStamped chosen_frontier_pt;
		geometry_msgs::PoseArray neighbor_frontier_pts;

		chosen_frontier_pt.pose = frontier_queue[0]->pose;
		chosen_frontier_pt.header.stamp = ros::Time::now();
		chosen_frontier_pt.header.frame_id = "map";
		neighbor_frontier_pts.header.stamp = ros::Time::now();
		neighbor_frontier_pts.header.frame_id = "map";

		for (int i = 0; i < frontier_queue[0]->neighbors.size(); i++){
			neighbor_frontier_pts.poses.push_back(frontier_queue[0]->neighbors[i]->pose);
		}

		while (ros::ok()){
			frontier_pub.publish(chosen_frontier_pt);
			neighbors_pub.publish(neighbor_frontier_pts);
		}
	}


};


int main(int argc, char **argv){

	ros::init(argc, argv, "frontier_choice_node");
	ros::NodeHandle nh_;

	ChooseFrontier choose_frontier;
	ros::ServiceServer choice_service = nh_.advertiseService("/choose_frontier", &ChooseFrontier::ServiceCallback, &choose_frontier);

	ros::spin();
	return 0;
}

