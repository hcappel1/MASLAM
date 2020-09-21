#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <frontier_pkg/FrontierMsg.h>


using namespace std;

class Node{

    public:

        Node(){
        	CommonInitialization();
        } 
        
        double x_pos;
        double y_pos;
        string key;
        bool map_open;
        bool frontier_open;
        int map_val;
        vector<int> neighbors;

        void CommonInitialization(){
        	x_pos = 0.0;
        	y_pos = 0.0;
        	key = (to_string(x_pos) + "-" + to_string(y_pos));
        	map_open = NULL;
        	frontier_open = NULL;
        }

};

class Frontier{

private:

	ros::NodeHandle nh;

	vector< shared_ptr<Node> > map_node;

    vector< shared_ptr<Node> > map_queue;
    vector< shared_ptr<Node> > frontier_queue; 
    vector< shared_ptr<Node> > new_frontier;

    shared_ptr<Node> current_node_map;
    shared_ptr<Node> current_node_frontier;
    shared_ptr<Node> neighbor_node_map;
    shared_ptr<Node> neighbor_node_frontier;

    vector<vector< shared_ptr<Node> > > frontier_list;
    geometry_msgs::Pose init_pose;

public:

	vector<signed char> map_raw;
	
	Frontier(){
		ROS_INFO("created frontier object");
	}

	bool MapCallback(frontier_pkg::FrontierMsg::Request &req,
					 frontier_pkg::FrontierMsg::Response &res)
	{
		map_raw = req.map_data.data;
		MapConvert();
		res.success = true;

		return true;
	}

	void MapConvert()
	{
		
	    for (int i = 0; i < map_raw.size(); i++){
	    	shared_ptr<Node> new_node = shared_ptr<Node>( new Node );
	    	new_node->x_pos = 1.0;
	    	new_node->y_pos = 1.0;
	    	new_node->key = (to_string(new_node->x_pos) + "-" + to_string(new_node->y_pos));
	    	new_node->map_val = int(map_raw[i]);
	    	map_node.push_back(new_node);
	    }
	}


};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_pts_node");
	ros::NodeHandle nh_;
	Frontier frontier;
	ros::ServiceServer map_service = nh_.advertiseService("/frontier_pts", &Frontier::MapCallback, &frontier);
	
	ros::spin();

	return 0;
}