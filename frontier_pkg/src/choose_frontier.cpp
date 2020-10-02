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
		SourceInitialization();
	}

	geometry_msgs::Pose pose;
	vector< shared_ptr<FrontierPt> > neighbors;
	bool valid;
	string key; 

	void SourceInitialization(){
		pose.position.x = 3.5;
		pose.position.y = 12.0;
		valid = false;
		key = "source";
	}
};

class ChooseFrontier{

private:
	ros::NodeHandle nh;
	ros::Publisher frontier_pub;
	ros::Publisher frontier_pts_pub;
	ros::Publisher valid_frontier_pts_pub;

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
		frontier_pts_pub = nh.advertise<geometry_msgs::PoseArray>("frontier_queue", 1000);
		valid_frontier_pts_pub = nh.advertise<geometry_msgs::PoseArray>("valid_frontier_queue", 1000);

	}

	bool ServiceCallback(frontier_pkg::ChoiceMsg::Request &req,
						frontier_pkg::ChoiceMsg::Response &res)
	{
		optimal_frontier_pts = req.optimal_frontier_pts;
		CreateFrontierQueue();
		GetNeighbors();
		ComputePaths(frontier_queue[0]);
		cout << "number of paths: " << source_paths.size() << endl;

		res.success = true;
	}

	void CreateFrontierQueue(){

		for (int i = 0; i < optimal_frontier_pts.poses.size(); i++){

			shared_ptr<FrontierPt> new_frontier_pt = shared_ptr<FrontierPt>( new FrontierPt );
			new_frontier_pt->pose = optimal_frontier_pts.poses[i];
			new_frontier_pt->pose.orientation.w = 1.0;
			new_frontier_pt->valid = false;
			new_frontier_pt->key = (to_string(optimal_frontier_pts.poses[i].position.x) + "-" + to_string(optimal_frontier_pts.poses[i].position.y));
			frontier_queue.push_back(new_frontier_pt);
		}
		shared_ptr<FrontierPt> source = shared_ptr<FrontierPt>( new FrontierPt );
		frontier_queue.push_back(source);
	}

	void GetNeighbors(){

		for (int i = 0; i < frontier_queue.size(); i++){
			for (int j = 0; j < frontier_queue.size(); j++){

				if (frontier_queue[i]->key != frontier_queue[j]->key){

					double distance = sqrt(pow(frontier_queue[i]->pose.position.x - frontier_queue[j]->pose.position.x,2) + pow(frontier_queue[i]->pose.position.y - frontier_queue[j]->pose.position.y,2));
					if (distance < 2.0){
						frontier_queue[i]->neighbors.push_back(frontier_queue[j]);
					}
				}
			}
		}
	}

	bool FrontierValid( shared_ptr<FrontierPt> frontier_pt){

		for (int i = 0; i < frontier_pt->neighbors.size(); i++){

			if (frontier_pt->neighbors[i]->key == "source"){
				return true;
			}
			else{
				for (int j = 0; j < frontier_pt->neighbors[i]->neighbors.size(); j++){

					if (frontier_pt->neighbors[i]->neighbors[j]->key == "source"){
						return true;
					}
					else{
						for (int k = 0; k < frontier_pt->neighbors[i]->neighbors[j]->neighbors.size(); k++){
							if (frontier_pt->neighbors[i]->neighbors[j]->neighbors[k]->key == "source"){
								return true;
							}
							else{
								continue;
							}
						}
					}
				}
			}
			
		}
		return false;
	}

	void ComputePaths( shared_ptr<FrontierPt> candidate_pt){
		vector< shared_ptr<FrontierPt> > source_path;
		source_path.push_back(candidate_pt);

		for (int i = 0; i < candidate_pt->neighbors.size(); i++){

			vector< shared_ptr<FrontierPt> > source_path_one = source_path;
			source_path_one.push_back(candidate_pt->neighbors[i]);

			if (candidate_pt->neighbors[i]->key == "source"){
				source_paths.push_back(source_path_one);
			}
			
			for (int j = 0; j < candidate_pt->neighbors[i]->neighbors.size(); j++){

				vector< shared_ptr<FrontierPt> > source_path_two = source_path_one;
				source_path_two.push_back(candidate_pt->neighbors[i]->neighbors[j]);

				if (candidate_pt->neighbors[i]->neighbors[j]->key == "source"){
					source_paths.push_back(source_path_two);
				}

				for (int k = 0; k < candidate_pt->neighbors[i]->neighbors[j]->neighbors.size(); k++){

					vector< shared_ptr<FrontierPt> > source_path_three = source_path_two;
					source_path_three.push_back(candidate_pt->neighbors[i]->neighbors[j]->neighbors[k]);
					
					if (candidate_pt->neighbors[i]->neighbors[j]->neighbors[k]->key == "source"){
						source_paths.push_back(source_path_three);
				
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
			//neighbors_pub.publish(neighbor_frontier_pts);
		}
	}

	void FrontierValidTest(){

		geometry_msgs::PoseArray valid_frontier_queue;
		valid_frontier_queue.header.stamp = ros::Time::now();
		valid_frontier_queue.header.frame_id = "map";
		optimal_frontier_pts.header.stamp = ros::Time::now();
		optimal_frontier_pts.header.frame_id = "map";



		for (int i = 0; i < frontier_queue.size(); i++){
			bool valid = FrontierValid(frontier_queue[i]);
			if (valid == true){
				cout << "frontier point valid" << endl;
				valid_frontier_queue.poses.push_back(frontier_queue[i]->pose);
			}
		}

		while (ros::ok()){
			frontier_pts_pub.publish(optimal_frontier_pts);
			valid_frontier_pts_pub.publish(valid_frontier_queue);
		}


	}

	void ComputePathTest(){
		
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

