// Service which generates a set of training data to train abstraction of hybrid driving-stepping locomotion
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#include <planning_abstraction_net/training_data_generator.h>


namespace planning_abstraction_net
{

TrainingDataGenerator::TrainingDataGenerator()
{
	m_map_size = 1.8f;
	m_resolution_detailed = 0.025f;
	m_resolution_abstract = 0.1f;
	m_map_size_cells = (int)(m_map_size / m_resolution_detailed + 1e-3);
	m_num_orientations = 64;

	m_nh = ros::NodeHandle("~");
	m_srv_generate_training_data_from_random = m_nh.advertiseService("/planning_abstraction_net/generate_training_data_from_random", &TrainingDataGenerator::GenerateTrainingDataServiceCall, this);
	m_srv_client_get_path_costs = m_nh.serviceClient<GetPathCosts>("/planner/get_path_costs");

	m_num_random_maps_1_obstacle = 1;
	m_num_random_maps_2_obstacle = 0;
	m_num_random_maps_3_obstacle = 0;
	m_num_random_maps_1_wall = 0;
	m_num_random_maps_2_wall = 0;
	m_num_random_maps_1_wall_1_obstacle = 0;
	m_num_random_maps_stairs = 0;
	m_num_random_maps_1_wall_stairs = 0;
	m_num_random_maps_stairs_focussed = 0;

	m_file_path = "/home/klamt/new_data_set.txt";

	srand(time(NULL));
}


bool TrainingDataGenerator::GenerateTrainingDataServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	std::vector<cv::Mat_<float>> height_maps;
	std::vector<std::vector<std::pair<RobotPose, RobotPose>>> abstract_poses;
	std::vector<std::vector<std::pair<RobotPose, RobotPose>>> detailed_poses;
	std::vector<std::vector<float>> costs;

	ROS_INFO("* Generate Maps *");
	GetRandomHeightMaps(height_maps);
	ROS_INFO_STREAM("  - " << height_maps.size() << " maps generated");


	ROS_INFO("* Generate Abstract Poses *");
	GenerateAbstractPoses(abstract_poses, height_maps);

	ROS_INFO("* Transform To Detailed Poses *");
	TransformAbstractPosesToDetailedPoses(height_maps, abstract_poses, detailed_poses);

	ROS_INFO("* Delete Tasks With Infeasible Start *");
	int num_deleted_maps = DeleteTasksWithInfeasibleStart(detailed_poses,
							      abstract_poses,
							      height_maps);
	ROS_INFO_STREAM("  - " << num_deleted_maps << " maps deleted");

	ROS_INFO("* Generate Costs *");
	GeneratePathCosts(height_maps, detailed_poses, costs);

	ROS_INFO("* Save Data *");
	m_data_serializer.SerializeData(height_maps, abstract_poses, costs, m_file_path);

	ROS_INFO("* Done! *");

	return true;
}


void TrainingDataGenerator::GetRandomHeightMaps(std::vector<cv::Mat_<float>>& height_maps) const
{
// 	Add random maps with one obstacles
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_1_obstacle; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomCuboid(0.2, 2, 0.02, 0.4);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

// 	Add random maps with two obstacles
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_2_obstacle; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomCuboid(0.2, 2, 0.02, 0.4);
		height_map_generator.AddRandomCuboid(0.2, 2, 0.02, 0.4);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

// 	Add random maps with three obstacles
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_3_obstacle; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomCuboid(0.2, 2, 0.02, 0.4);
		height_map_generator.AddRandomCuboid(0.2, 2, 0.02, 0.4);
		height_map_generator.AddRandomCuboid(0.2, 2, 0.02, 0.4);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

	// Add random maps with one wall
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_1_wall; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomWall(0.4, 3.0, 0.05, 0.35, 0.7, 2.0);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

	// Add random maps with two walls
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_2_wall; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomWall(0.4, 3.0, 0.05, 0.35, 0.7, 2.0);
		height_map_generator.AddRandomWall(0.4, 3.0, 0.05, 0.35, 0.7, 2.0);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

	// Add random maps with one wall and one obstacle
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_1_wall_1_obstacle; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomWall(0.4, 3.0, 0.05, 0.35, 0.7, 2.0);
		height_map_generator.AddRandomCuboid(0.2, 2, 0.02, 0.4);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

	// Add stair with random position and orientation and random number of steps
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_stairs; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomStair(2, 4, 0.15, 0.3, 0.25, 0.35, 0.9, 2, -M_PI, M_PI);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

	// Add random stair with one wall
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_1_wall_stairs; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		height_map_generator.AddRandomWall(0.4, 3.0, 0.05, 0.35, 0.7, 2.0);
		height_map_generator.AddRandomStair(2, 4, 0.15, 0.3, 0.25, 0.35, 0.9, 2, -M_PI, M_PI);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}

	// Add stair with random position and but orientation within a given orientation interval
	for (int map_gen_counter = 0; map_gen_counter < m_num_random_maps_stairs_focussed; map_gen_counter++)
	{
		HeightMapGeneratorAbstrLearning height_map_generator(m_map_size, m_resolution_detailed);
		height_map_generator.AddGroundPlane();
		float orientation_step = 2 * M_PI / m_num_orientations;
		height_map_generator.AddRandomStair(2, 4, 0.15, 0.3, 0.25, 0.35, 1.4, 2, -2 * orientation_step, 2 * orientation_step);
		height_maps.push_back(height_map_generator.GetHeightMap());
	}
}


void TrainingDataGenerator::GenerateAbstractPoses(std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& poses,
						  std::vector<cv::Mat_<float>>& height_maps) const
{
	std::vector<std::pair<RobotPose, RobotPose>> current_pose_vector;

	// Create start pose with it base center at the map center and fixed orientation. There are no individual foot positions abstract poses.
	RobotPose start_pose;
	start_pose.base_coordinate = Eigen::Vector2f(0.0f, 0.0f);
	start_pose.orientation = 0.0f;
	start_pose.rel_wheel_coordinate_fl = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	start_pose.rel_wheel_coordinate_bl = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	start_pose.rel_wheel_coordinate_br = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	start_pose.rel_wheel_coordinate_fr = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	start_pose.abs_wheel_coordinate_fl = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	start_pose.abs_wheel_coordinate_bl = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	start_pose.abs_wheel_coordinate_br = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
	start_pose.abs_wheel_coordinate_fr = Eigen::Vector2f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());


	// Create goal poses which is done by either omnidirectional driving within the used 20-neighborhood or by turning on the place
	// Vector of movements in x,y,yaw (cell, cell, orientation step)
	std::vector<Eigen::Vector3i> movement_vector;
	movement_vector.push_back(Eigen::Vector3i( 1, 0, 0));
	movement_vector.push_back(Eigen::Vector3i( 1, 1, 0));
	movement_vector.push_back(Eigen::Vector3i( 0, 1, 0));
	movement_vector.push_back(Eigen::Vector3i(-1, 1, 0));
	movement_vector.push_back(Eigen::Vector3i(-1, 0, 0));
	movement_vector.push_back(Eigen::Vector3i(-1,-1, 0));
	movement_vector.push_back(Eigen::Vector3i( 0,-1, 0));
	movement_vector.push_back(Eigen::Vector3i( 1,-1, 0));
	movement_vector.push_back(Eigen::Vector3i( 2, 0, 0));
	movement_vector.push_back(Eigen::Vector3i( 2, 1, 0));
	movement_vector.push_back(Eigen::Vector3i( 1, 2, 0));
	movement_vector.push_back(Eigen::Vector3i( 0, 2, 0));
	movement_vector.push_back(Eigen::Vector3i(-1, 2, 0));
	movement_vector.push_back(Eigen::Vector3i(-2, 1, 0));
	movement_vector.push_back(Eigen::Vector3i(-2, 0, 0));
	movement_vector.push_back(Eigen::Vector3i(-2,-1, 0));
	movement_vector.push_back(Eigen::Vector3i(-1, -2, 0));
	movement_vector.push_back(Eigen::Vector3i( 0, -2, 0));
	movement_vector.push_back(Eigen::Vector3i( 1, -2, 0));
	movement_vector.push_back(Eigen::Vector3i( 2, -1, 0));
	movement_vector.push_back(Eigen::Vector3i( 0, 0, 1));
	movement_vector.push_back(Eigen::Vector3i( 0, 0, -1));


	float orientation_step = 2 * M_PI / 16;

	for (Eigen::Vector3i movement : movement_vector)
	{
		RobotPose goal_pose = start_pose;
		Eigen::Vector2f absolute_position = Eigen::Vector2f(movement.x() * m_resolution_abstract, movement.y() * m_resolution_abstract);
		float absolute_orientation = movement.z() * orientation_step;
		goal_pose.base_coordinate = absolute_position;
		goal_pose.orientation = absolute_orientation;
		current_pose_vector.push_back(std::pair<RobotPose, RobotPose>(start_pose, goal_pose));
	}


	for (uint i = 0; i < height_maps.size(); i++)
	{
		poses.push_back(current_pose_vector);
	}
}


void TrainingDataGenerator::TransformAbstractPosesToDetailedPoses(const std::vector<cv::Mat_<float>>& height_maps,
								  const std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& abstract_poses,
								  std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& detailed_poses) const
{
	for (uint map_counter = 0; map_counter < abstract_poses.size(); map_counter++)
	{
		std::vector<std::pair<RobotPose, RobotPose>> map_vector;

		for (uint task_counter = 0; task_counter < abstract_poses[map_counter].size(); task_counter++)
		{
			RobotPose abstract_start_pose = abstract_poses[map_counter][task_counter].first;
			RobotPose abstract_goal_pose = abstract_poses[map_counter][task_counter].second;

			RobotPose detailed_start_pose = TransformAbstractPoseToDetailedPose(height_maps[map_counter],
											    abstract_start_pose);
			RobotPose detailed_goal_pose = TransformAbstractPoseToDetailedPose(height_maps[map_counter],
											    abstract_goal_pose);

			map_vector.push_back(std::pair<RobotPose, RobotPose>(detailed_start_pose, detailed_goal_pose));
		}
		detailed_poses.push_back(map_vector);
	}
}


RobotPose TrainingDataGenerator::TransformAbstractPoseToDetailedPose(const cv::Mat_<float>& height_map,
								       const RobotPose& abstract_pose) const
{
	// This transformation is specific on your robot representation and needs to be specified by you.
	// To obtain a working example code, I just return the input pose.
	RobotPose detailed_pose =  abstract_pose;
	detailed_pose.rel_wheel_coordinate_fl = Eigen::Vector2f(0.3f, 0.2f);
	detailed_pose.rel_wheel_coordinate_bl = Eigen::Vector2f(-0.3f, 0.2f);
	detailed_pose.rel_wheel_coordinate_br = Eigen::Vector2f(-0.3f, -0.2f);
	detailed_pose.rel_wheel_coordinate_fr = Eigen::Vector2f(0.3f, -0.2f);
	FillAbsoluteWheelCoordinates(detailed_pose);
	return detailed_pose;
}


int TrainingDataGenerator::DeleteTasksWithInfeasibleStart(std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& detailed_poses,
							  std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& abstract_poses,
							  std::vector<cv::Mat_<float>>& height_maps)
{
	uint map_index = 0;
	int delete_counter = 0;

	while(true)
	{
		RobotPose current_start_pose = detailed_poses[map_index][0].first;

		// If the detailed start pose for the current map is infeasible
		// (which is encoded by infinit coordinates of the respective robot elements)
		if (std::fabs(current_start_pose.base_coordinate.x()) > 1000.0f
		 || std::fabs(current_start_pose.rel_wheel_coordinate_fl.x()) > 1000.0f
		 || std::fabs(current_start_pose.rel_wheel_coordinate_bl.x()) > 1000.0f
		 || std::fabs(current_start_pose.rel_wheel_coordinate_br.x()) > 1000.0f
		 || std::fabs(current_start_pose.rel_wheel_coordinate_fr.x()) > 1000.0f)
		{
			delete_counter++;
			detailed_poses.erase(detailed_poses.begin() + map_index);
			abstract_poses.erase(abstract_poses.begin() + map_index);
			height_maps.erase(height_maps.begin() + map_index);
		}
		else
		{
			map_index++;
		}

		// If this was the last map, break
		if (map_index == height_maps.size())
			break;
	}

	return delete_counter;
}


void TrainingDataGenerator::GeneratePathCosts(const std::vector<cv::Mat_<float>>& height_maps,
					      const std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& detailed_poses,
					      std::vector<std::vector<float>>& costs)
{
	for (uint map_counter = 0; map_counter < height_maps.size(); map_counter++)
	{
		if (map_counter%100 == 99)
			ROS_INFO_STREAM("   Map counter: " << map_counter+1);

		std::vector<float> cost_vector;

		// generate vectors of maps for service request
		std::vector<float> heights;

		int index_counter = 0;
		for (int y = 0; y < m_map_size_cells; y++)
			for (int x = 0; x < m_map_size_cells; x++)
				heights.push_back(height_maps[map_counter](x,y));


		for (uint task_counter = 0; task_counter < detailed_poses[map_counter].size(); task_counter++)
		{
			RobotPose detailed_start_pose = detailed_poses[map_counter][task_counter].first;
			RobotPose detailed_goal_pose = detailed_poses[map_counter][task_counter].second;

			// If either start or goal pose are not feasible, asign infinite costs
			if (std::fabs(detailed_start_pose.base_coordinate.x()) > 1000.0f
			 || std::fabs(detailed_start_pose.rel_wheel_coordinate_fl.x()) > 1000.0f
			 || std::fabs(detailed_start_pose.rel_wheel_coordinate_bl.x()) > 1000.0f
			 || std::fabs(detailed_start_pose.rel_wheel_coordinate_br.x()) > 1000.0f
			 || std::fabs(detailed_start_pose.rel_wheel_coordinate_fr.x()) > 1000.0f
			 || std::fabs(detailed_goal_pose.base_coordinate.x()) > 1000.0f
			 || std::fabs(detailed_goal_pose.rel_wheel_coordinate_fl.x()) > 1000.0f
			 || std::fabs(detailed_goal_pose.rel_wheel_coordinate_bl.x()) > 1000.0f
			 || std::fabs(detailed_goal_pose.rel_wheel_coordinate_br.x()) > 1000.0f
			 || std::fabs(detailed_goal_pose.rel_wheel_coordinate_fr.x()) > 1000.0f)
			{
				cost_vector.push_back(std::numeric_limits<float>::infinity());
			}
			else
			{
				GetPathCosts srv;
				srv.request.map_size_cells = m_map_size_cells;
				srv.request.resolution = m_resolution_detailed;
				srv.request.origin_x = -0.5 * m_map_size;
				srv.request.origin_y = -0.5 * m_map_size;
				srv.request.heights = heights;

				srv.request.start_x = detailed_start_pose.base_coordinate.x();
				srv.request.start_y = detailed_start_pose.base_coordinate.y();
				srv.request.start_orient = detailed_start_pose.orientation;
				srv.request.start_rel_foot_fl_x = detailed_start_pose.rel_wheel_coordinate_fl.x();
				srv.request.start_rel_foot_fl_y = detailed_start_pose.rel_wheel_coordinate_fl.y();
				srv.request.start_rel_foot_bl_x = detailed_start_pose.rel_wheel_coordinate_bl.x();
				srv.request.start_rel_foot_bl_y = detailed_start_pose.rel_wheel_coordinate_bl.y();
				srv.request.start_rel_foot_br_x = detailed_start_pose.rel_wheel_coordinate_br.x();
				srv.request.start_rel_foot_br_y = detailed_start_pose.rel_wheel_coordinate_br.y();
				srv.request.start_rel_foot_fr_x = detailed_start_pose.rel_wheel_coordinate_fr.x();
				srv.request.start_rel_foot_fr_y = detailed_start_pose.rel_wheel_coordinate_fr.y();

				srv.request.goal_x = detailed_goal_pose.base_coordinate.x();
				srv.request.goal_y = detailed_goal_pose.base_coordinate.y();
				srv.request.goal_orient = detailed_goal_pose.orientation;
				srv.request.goal_rel_foot_fl_x = detailed_goal_pose.rel_wheel_coordinate_fl.x();
				srv.request.goal_rel_foot_fl_y = detailed_goal_pose.rel_wheel_coordinate_fl.y();
				srv.request.goal_rel_foot_bl_x = detailed_goal_pose.rel_wheel_coordinate_bl.x();
				srv.request.goal_rel_foot_bl_y = detailed_goal_pose.rel_wheel_coordinate_bl.y();
				srv.request.goal_rel_foot_br_x = detailed_goal_pose.rel_wheel_coordinate_br.x();
				srv.request.goal_rel_foot_br_y = detailed_goal_pose.rel_wheel_coordinate_br.y();
				srv.request.goal_rel_foot_fr_x = detailed_goal_pose.rel_wheel_coordinate_fr.x();
				srv.request.goal_rel_foot_fr_y = detailed_goal_pose.rel_wheel_coordinate_fr.y();


				if (m_srv_client_get_path_costs.call(srv))
					cost_vector.push_back((float)(srv.response.cost));
				else
					throw std::logic_error("Service could not be called");
			}
		}
		costs.push_back(cost_vector);
	}
}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "training_data_generator");
	planning_abstraction_net::TrainingDataGenerator td_generator;
	ros::spin();
	return 0;
}
