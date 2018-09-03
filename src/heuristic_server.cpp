// performs a Dijkstra search on wheel cost to all valid map cells, starting from the goal cell. Cost are provided as wheel heuristics
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2017 

#include <planning_abstraction_net/heuristic_server.h>

namespace planning_abstraction_net
{
HeuristicServer::HeuristicServer()
{
	m_resolution_detailed = 0.025f;
	m_resolution_abstract = 0.1f;
	m_num_cells_crop = 72;
	m_num_orientations_abstract = 16;

	m_nh = ros::NodeHandle("~");
	m_srv_client_set_map_crops = m_nh.serviceClient<planning_abstraction_net::SetMapCrops>("/abstraction_learning_network/set_map_crops");
	m_srv_client_get_abstraction = m_nh.serviceClient<planning_abstraction_net::GetBatchAbstraction>("/abstraction_learning_network/get_batch_abstraction");

	m_action_list.push_back(Eigen::Vector3i(1, 0, 0));
	m_action_list.push_back(Eigen::Vector3i(1, 1, 0));
	m_action_list.push_back(Eigen::Vector3i(0, 1, 0));
	m_action_list.push_back(Eigen::Vector3i(-1, 1, 0));
	m_action_list.push_back(Eigen::Vector3i(-1, 0, 0));
	m_action_list.push_back(Eigen::Vector3i(-1, -1, 0));
	m_action_list.push_back(Eigen::Vector3i(0, -1, 0));
	m_action_list.push_back(Eigen::Vector3i(1, -1, 0));
	m_action_list.push_back(Eigen::Vector3i(2, 0, 0));
	m_action_list.push_back(Eigen::Vector3i(2, 1, 0));
	m_action_list.push_back(Eigen::Vector3i(1, 2, 0));
	m_action_list.push_back(Eigen::Vector3i(0, 2, 0));
	m_action_list.push_back(Eigen::Vector3i(-1, 2, 0));
	m_action_list.push_back(Eigen::Vector3i(-2, 1, 0));
	m_action_list.push_back(Eigen::Vector3i(-2, 0, 0));
	m_action_list.push_back(Eigen::Vector3i(-2, -1, 0));
	m_action_list.push_back(Eigen::Vector3i(-1, -2, 0));
	m_action_list.push_back(Eigen::Vector3i(0, -2, 0));
	m_action_list.push_back(Eigen::Vector3i(1, -2, 0));
	m_action_list.push_back(Eigen::Vector3i(2, -1, 0));
	m_action_list.push_back(Eigen::Vector3i(0, 0, 1));
	m_action_list.push_back(Eigen::Vector3i(0, 0, -1));
}


void HeuristicServer::Initialize(cv::Mat_<float>& height_map)
{
	m_height_map = height_map;
	m_num_cells_x_detailed = m_height_map.cols;
	m_num_cells_y_detailed = m_height_map.rows;
	m_num_cells_x_abstract = (int)(m_num_cells_x_detailed * m_resolution_detailed / m_resolution_abstract + 1e-3);
	m_num_cells_y_abstract = (int)(m_num_cells_y_detailed * m_resolution_detailed / m_resolution_abstract + 1e-3);
	m_map_crop_size = m_num_cells_crop * m_resolution_detailed;
	float orientation_step = 2 * M_PI / m_num_orientations_abstract;

	// m_pose_data contains relevant data for each abstract pose in the map (e.g., g_costs/heuristic to the goal)
	m_pose_data = std::vector<DiscrPoseData>(m_num_cells_x_abstract * m_num_cells_y_abstract * m_num_orientations_abstract);
//
	// Generate a map crop for each abstract pose in the map and send those map crops to the PyTorch CNN interface
	// Since large amounts of data are generated, map crops are sent in chunks
	for (uint orient = 0; orient < m_num_orientations_abstract; orient++)
	{
		std::vector<float> map_crops;
		std::vector<uint> map_crops_x;
		std::vector<uint> map_crops_y;
		std::vector<uint> map_crops_orientations;
		for (uint y = 0; y < m_num_cells_y_abstract; y++)
		{
			for (uint x = 0; x < m_num_cells_x_abstract; x++)
			{
				Eigen::Vector2f absolute_start_position = CellToCoordinates(Eigen::Vector2i(x,y), m_resolution_abstract);
				float absolute_start_orientation = orient * orientation_step;
				map_crops_x.push_back(x);
				map_crops_y.push_back(y);
				map_crops_orientations.push_back(orient);
				std::vector<float> map_crop = GetHeightMapCrop(absolute_start_position, absolute_start_orientation);
				map_crops.insert(map_crops.end(), map_crop.begin(), map_crop.end());
			}
		}

		planning_abstraction_net::SetMapCrops srv;
		srv.request.num_crops_x = m_num_cells_x_abstract;
		srv.request.num_crops_y = m_num_cells_y_abstract;
		srv.request.num_crops_orient = m_num_orientations_abstract;
		srv.request.crop_size = m_map_crop_size;

		srv.request.crop_x = map_crops_x;
		srv.request.crop_y = map_crops_y;
		srv.request.crop_orient = map_crops_orientations;
		srv.request.heights = map_crops;

		if (m_srv_client_set_map_crops.call(srv))
		{
		}
		else
			throw std::logic_error("Service could not be called");

		ROS_INFO_STREAM("Map crops set for orientation " << orient);
	}

	// For each abstract pose on the map, generate its potential neighbors. Since the heuristic computations starts from the planner goal pose,
	// it is running backwards. Hence "inverse" neighbors are generated. Note that our cost function prefers driving forward and thus inverse actions may carry different costs.
	m_inverse_neighbors = std::vector<std::vector<std::pair<Eigen::Vector3i, float>>> (m_num_cells_x_abstract * m_num_cells_y_abstract * m_num_orientations_abstract);
	std::vector<Eigen::Vector3i> discr_poses;
	for (uint orient = 0; orient < m_num_orientations_abstract; orient++)
	{
		for (uint y = 0; y < m_num_cells_y_abstract; y++)
		{
			for (uint x = 0; x < m_num_cells_x_abstract; x++)
			{
				discr_poses.push_back(Eigen::Vector3i(x,y,orient));
			}
		}
	}
	GetNeighboursFromAbstraction(discr_poses);
}


void HeuristicServer::SetGoal(const RobotPose& goal_pose)
{
	Clear();

	// Fit the goal pose to the abstract representation and start the backward one-to-any Dijkstra search
	m_goal_pose = goal_pose;
	m_goal_pose.base_coordinate = CellToCoordinates(CoordinatesToCell(m_goal_pose.base_coordinate, m_resolution_abstract), m_resolution_abstract);
	FitOrientationToGrid(m_goal_pose.orientation, m_num_orientations_abstract);
	FillAbsoluteWheelCoordinates(m_goal_pose);
	FillHeuristics();
}


void HeuristicServer::Clear ()
{
	m_priority_queue.Clear();
	m_pose_data = std::vector<DiscrPoseData>(m_num_cells_x_abstract * m_num_cells_y_abstract * m_num_orientations_abstract);
}


float HeuristicServer::GetHeuristic(const RobotPose& pose) const
{
	Eigen::Vector2i cell = CoordinatesToCell(pose.base_coordinate, m_resolution_abstract);
	int discr_orientation = GetDiscreteOrientation(pose.orientation, m_num_orientations_abstract);
	return m_pose_data[Index(Eigen::Vector3i(cell.x(), cell.y(), discr_orientation))].g_cost;
}

// One-to-any Dijkstra search to fill heuristics, starting from the set goal pose and accessing the precomputed neighbors and costs.
void HeuristicServer::FillHeuristics()
{
	clock_t start_time = clock();

	DiscrPoseData start_data;
	start_data.g_cost = 0.0f;
	start_data.in_open_list = true;
	Eigen::Vector2i start_cell = CoordinatesToCell(m_goal_pose.base_coordinate, m_resolution_abstract);
	int start_discr_orientation = GetDiscreteOrientation(m_goal_pose.orientation, m_num_orientations_abstract);
	Eigen::Vector3i start_discr_data = Eigen::Vector3i(start_cell.x(), start_cell.y(), start_discr_orientation);
	m_priority_queue.push(start_discr_data, 0.0f);
	m_pose_data[Index(start_discr_data)] = start_data;

	while(m_priority_queue.size() > 0)
	{
		std::pair<Eigen::Vector3i, float> top_entry = m_priority_queue.top();
		Eigen::Vector3i current_discr_pose = top_entry.first;
		float current_cost = top_entry.second;
		m_priority_queue.pop();

		DiscrPoseData& current_discr_pose_data = m_pose_data[Index(current_discr_pose)];
		current_discr_pose_data.in_open_list = false;
		current_discr_pose_data.in_closed_list = true;

		std::vector<std::pair<Eigen::Vector3i, float>> neighbors = m_inverse_neighbors[Index(current_discr_pose)];
		for (std::pair<Eigen::Vector3i, float>& neighbor : neighbors)
		{
			float new_cost = current_cost + neighbor.second;

			DiscrPoseData& current_neighbor_data = m_pose_data[Index(neighbor.first)];

			if (!current_neighbor_data.in_closed_list)
			{
				if (!current_neighbor_data.in_open_list)
				{
					current_neighbor_data.in_open_list = true;
					current_neighbor_data.g_cost = new_cost;
					current_neighbor_data.predecessor = current_discr_pose;
					current_neighbor_data.priority_queue_handle = m_priority_queue.push(neighbor.first, new_cost);
				}
				else
				{
					if (new_cost < current_neighbor_data.g_cost)
					{
						current_neighbor_data.g_cost = new_cost;
						current_neighbor_data.predecessor = current_discr_pose;
						m_priority_queue.DecreaseKey(neighbor.first, new_cost, current_neighbor_data.priority_queue_handle);
					}
				}
			}
		}
	}
	ROS_INFO_STREAM("Filling the heuristic took " << (float)(clock()-start_time) / CLOCKS_PER_SEC);
}


// For each given abstract pose, compute potential inverse neighbors and ask the CNN about feasibility and costs (via ROS-Service)
void HeuristicServer::GetNeighboursFromAbstraction(const std::vector<Eigen::Vector3i>& discr_poses)
{
	float orientation_step = 2 * M_PI / m_num_orientations_abstract;

	std::vector<uint> map_crop_indices;
	std::vector<float> goals_x;
	std::vector<float> goals_y;
	std::vector<float> goals_orient;

	uint batch_size = 0;

	for (Eigen::Vector3i discr_pose : discr_poses)
	{
		// Skip map border where some actions would result in poses outside of the map
		if (discr_pose.x() < 2
		 || discr_pose.x() > m_num_cells_x_abstract - 3
		 || discr_pose.y() < 2
		 || discr_pose.y() > m_num_cells_y_abstract - 3)
		{
			continue;
		}

		for (int i = 0; i < m_action_list.size(); i++)
		{
			Eigen::Vector3i action = m_action_list[i];
			Eigen::Vector3i discr_start_pose = discr_pose - action;
			if (discr_start_pose.z() < 0)
				discr_start_pose.z() += m_num_orientations_abstract;
			if (discr_start_pose.z() >= m_num_orientations_abstract)
				discr_start_pose.z() -= m_num_orientations_abstract;

			map_crop_indices.push_back(Index(discr_start_pose));

			// get discrete normalized goal
			float absolute_start_orientation = discr_start_pose.z() * orientation_step;
			Eigen::Rotation2D<float> inverse_rotation_2D(-absolute_start_orientation);
			Eigen::Vector2f normalized_goal_pos = inverse_rotation_2D * Eigen::Vector2f(action.x(), action.y());
			goals_x.push_back(normalized_goal_pos.x());
			goals_y.push_back(normalized_goal_pos.y());
			goals_orient.push_back(action.z());

			batch_size++;
		}
	}

	GetBatchAbstraction srv;
	srv.request.batch_size = batch_size;
	srv.request.map_crop_indices = map_crop_indices;
	srv.request.goals_x = goals_x;
	srv.request.goals_y = goals_y;
	srv.request.goals_orient = goals_orient;
	std::vector<float> costs;
	if (batch_size > 0)
	{
		if (m_srv_client_get_abstraction.call(srv))
			costs = srv.response.costs;
		else
			throw std::logic_error("Service could not be called");
	}


	// save inverse neighbors in data structure
	uint position_in_batch = 0;
	for (Eigen::Vector3i discr_pose : discr_poses)
	{
		m_inverse_neighbors[Index(discr_pose)] = {};

		if (discr_pose.x() < 2
		 || discr_pose.x() > m_num_cells_x_abstract - 3
		 || discr_pose.y() < 2
		 || discr_pose.y() > m_num_cells_y_abstract - 3)
			continue;

		for (int i = 0; i < m_action_list.size(); i++)
		{
			Eigen::Vector3i action = m_action_list[i];
			Eigen::Vector3i discr_start_pose = discr_pose - action;
			if (discr_start_pose.z() < 0)
				discr_start_pose.z() += m_num_orientations_abstract;
			if (discr_start_pose.z() >= m_num_orientations_abstract)
				discr_start_pose.z() -= m_num_orientations_abstract;

			float cost = costs[position_in_batch];
			position_in_batch++;

			if (cost < 1000.0f)
				m_inverse_neighbors[Index(discr_pose)].push_back(std::pair<Eigen::Vector3i, float>(discr_start_pose, cost));
		}
	}
}


// Extract a map crop from the given map at a given position with a given orientation.
std::vector<float> HeuristicServer::GetHeightMapCrop(const Eigen::Vector2f& position, const float& orientation) const
{
	Eigen::Rotation2D<float> rotation_2D (orientation);
	Eigen::Vector2f map_crop_origin = position + rotation_2D * Eigen::Vector2f(-0.5 * m_map_crop_size - 0.5 * m_resolution_abstract, -0.5 * m_map_crop_size - 0.5 * m_resolution_abstract);
	std::vector<float> map_crop(m_num_cells_crop * m_num_cells_crop);

	for (uint y_crop = 0; y_crop < m_num_cells_crop; y_crop++)
	{
		for (uint x_crop = 0; x_crop < m_num_cells_crop; x_crop++)
		{
			Eigen::Vector2f tmp_position = map_crop_origin + rotation_2D * Eigen::Vector2f((x_crop + 0.5) * m_resolution_abstract, (y_crop + 0.5) * m_resolution_abstract);
			Eigen::Vector2i tmp_cell = CoordinatesToCell(tmp_position, m_resolution_detailed);
			if (tmp_cell.x() >= 0
			 && tmp_cell.x() < m_num_cells_x_detailed
			 && tmp_cell.y() >= 0
			 && tmp_cell.y() < m_num_cells_y_detailed)
			{
				float height = m_height_map(tmp_cell.x(), tmp_cell.y());
				if (std::isnan(height))
					height = 2.0f;

				map_crop[x_crop + y_crop * m_num_cells_crop] = height;
			}
			else
				map_crop[x_crop + y_crop * m_num_cells_crop] = 2.0f;
		}
	}
	return map_crop;
}


int HeuristicServer::Index(const Eigen::Vector3i& discr_pose) const
{
	int index = discr_pose.x() + discr_pose.y() * m_num_cells_x_abstract + discr_pose.z() * m_num_cells_x_abstract * m_num_cells_y_abstract;
	return index;
}


// Transformation from a map cell to the absolute coordinate
Eigen::Vector2f HeuristicServer::CellToCoordinates(const Eigen::Vector2i cell, const float resolution) const
{
	return (cell.cast<float>() * resolution) + Eigen::Vector2f(0.5 * resolution, 0.5 * resolution);
}


// Transformation from an absolute coordinate to the map cell
Eigen::Vector2i HeuristicServer::CoordinatesToCell(const Eigen::Vector2f coordinates, const float resolution) const
{
	return (coordinates / resolution).cast<int>();
}


int HeuristicServer::GetDiscreteOrientation(const float orientation, const unsigned int num_orientations) const
{
	return fmod(std::floor(orientation / (2.0f * M_PI) * num_orientations + 0.5f), num_orientations);
}
}
