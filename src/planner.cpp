// This is just a dummy planner which you can exchange with your detailed planner
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#include <planning_abstraction_net/planner.h>

namespace planning_abstraction_net
{
Planner::Planner()
{
	m_nh = ros::NodeHandle("~");

	m_srv_get_path_costs = m_nh.advertiseService("/planner/get_path_costs", &Planner::GetPathCostsCallback, this);
}


void Planner::Init()
{
	m_heuristic_server.Initialize(m_height_map);
}


bool Planner::GetPathCostsCallback(planning_abstraction_net::GetPathCosts::Request &request, planning_abstraction_net::GetPathCosts::Response &response)
{
	// This returns some dummy costs and should be replaced by your planner.
	// It should not use the abstraction based heuristic since this service call is used to learn the heuristic.

	response.cost = 0.0f;

	return true;
}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner");
	planning_abstraction_net::Planner planner;
	ros::spin();
	return 0;
}
