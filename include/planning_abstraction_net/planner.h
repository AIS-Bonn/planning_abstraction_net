// This is just a dummy planner which you can exchange with your detailed planner
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#ifndef PLANNER_H
#define PLANNER_H

#include <opencv2/opencv.hpp>
#include <planning_abstraction_net/GetPathCosts.h>
#include <planning_abstraction_net/heuristic_server.h>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>


namespace planning_abstraction_net
{

// The planner is an arbitrary traditional (search-based, sampling-based) planner for
// robot motion planning which uses a heuristic to guide planning. While for the planner itself
// only a framework is given, the heuristic is realized through a learned abstract representation
// and provided by the heuristic server.
class Planner
{
public:
	Planner();

	void Init();

private:
	bool GetPathCostsCallback(planning_abstraction_net::GetPathCosts::Request &request, planning_abstraction_net::GetPathCosts::Response &response);

	HeuristicServer m_heuristic_server;
	cv::Mat_<float> m_height_map;

	ros::NodeHandle m_nh;
	ros::ServiceServer m_srv_get_path_costs;

};
}


#endif // PLANNER_H
