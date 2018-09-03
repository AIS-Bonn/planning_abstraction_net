// performs a Dijkstra search on wheel cost to all valid map cells, starting from the goal cell. Cost are provided as wheel heuristics
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2017

#ifndef HEURISTIC_SERVER_H
#define HEURISTIC_SERVER_H

#include <opencv2/opencv.hpp>
#include <planning_abstraction_net/GetBatchAbstraction.h>
#include <planning_abstraction_net/SetMapCrops.h>
#include <planning_abstraction_net/priority_queue_discr_pose.h>
#include <planning_abstraction_net/robot_pose.h>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/policies.hpp>

#include <ros/node_handle.h>

namespace planning_abstraction_net
{
struct DiscrPoseData
{
	float g_cost = std::numeric_limits< float >::infinity();
	Eigen::Vector3i predecessor;
	bool in_open_list = false;
	bool in_closed_list = false;
	boost::heap::fibonacci_heap<std::pair<Eigen::Vector3i, float>, boost::heap::compare<heap_compare_discr_pose>>::handle_type priority_queue_handle;
};

class HeuristicServer
{
public:
	HeuristicServer();
	
	void Initialize(cv::Mat_<float>& cost_map);
	
	void SetGoal (const RobotPose& goal_pose);
	
	void Clear ();
	
	float GetHeuristic (const RobotPose& pose) const;
	
private:
	void FillHeuristics();

	void GetNeighboursFromAbstraction(const std::vector<Eigen::Vector3i>& discr_poses);

	std::vector<float> GetHeightMapCrop(const Eigen::Vector2f& position, const float& orientation) const;

	int Index(const Eigen::Vector3i& discr_pose) const;

	Eigen::Vector2f CellToCoordinates(const Eigen::Vector2i cell, const float resolution) const;

	Eigen::Vector2i CoordinatesToCell(const Eigen::Vector2f coordinates, const float resolution) const;

	int GetDiscreteOrientation(const float orientation, const unsigned int num_orientations) const;


	std::vector<Eigen::Vector3i> m_action_list;
	
	PriorityQueueDiscrPose m_priority_queue;
	
	cv::Mat_<float> m_height_map;
	
	ros::NodeHandle m_nh;
	ros::ServiceClient m_srv_client_set_map_crops;
	ros::ServiceClient m_srv_client_get_abstraction;

	int m_num_cells_x_detailed;
	int m_num_cells_y_detailed;
	int m_num_cells_x_abstract;
	int m_num_cells_y_abstract;
	int m_num_cells_crop;
	float m_resolution_detailed;
	float m_resolution_abstract;
	int m_num_orientations_abstract;
	float m_map_crop_size;
	
	std::vector<std::vector<float>> m_height_map_crops;
	std::vector<std::vector<std::pair<Eigen::Vector3i, float>>> m_inverse_neighbors;
	
	RobotPose m_goal_pose;
	
	std::vector<DiscrPoseData> m_pose_data;
};
}

#endif // HEURISTIC_SERVER_H
