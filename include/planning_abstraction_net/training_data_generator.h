// Service which generates a set of training data to train abstraction of hybrid driving-stepping locomotion
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#ifndef TRAINING_DATA_GENERATOR_H
#define TRAINING_DATA_GENERATOR_H

#include <planning_abstraction_net/data_serializer.h>
#include <planning_abstraction_net/height_map_generator_abstr_learning.h>
#include <planning_abstraction_net/GetPathCosts.h>
#include <planning_abstraction_net/robot_pose.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <time.h>
#include <ros/node_handle.h>

namespace planning_abstraction_net
{

// The TrainingDataGenerator generates data sets to train an abstract_representation_CNN.
// It generates random height maps of different categories and defines planning tasks from a
// start to a goal pose on these maps. The start pose is assumed to be in the robot center with
// a fixed orientation. The goal pose is one abstract action away from the start pose. Both start
// and goal poses are represented in three dimensions (x,y,yaw of the robot base). Other DoF are neglected.
// Tasks be solved by a planner to obtain the desired path costs. This requires poses to be transformed
// to the needed detailed planning representation. Finally data sets are serialized.
class TrainingDataGenerator
{
public:
	TrainingDataGenerator();

private:
	bool GenerateTrainingDataServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

	void GetRandomHeightMaps(std::vector<cv::Mat_<float>>& height_maps) const;

	void GenerateAbstractPoses(std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& poses, std::vector<cv::Mat_<float>>& height_maps) const;

	void TransformAbstractPosesToDetailedPoses(const std::vector<cv::Mat_<float>>& height_maps,
						   const std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& abstract_poses,
						   std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& detailed_poses) const;

	RobotPose TransformAbstractPoseToDetailedPose(const cv::Mat_<float>& height_map,
						      const RobotPose& abstract_pose) const;

	int DeleteTasksWithInfeasibleStart(std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& detailed_poses,
					   std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& abstract_poses,
				           std::vector<cv::Mat_<float>>& height_maps);

	void GeneratePathCosts(const std::vector<cv::Mat_<float>>& height_maps,
			       const std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& detailed_poses,
			       std::vector<std::vector<float>>& costs);

	ros::NodeHandle m_nh;
	ros::ServiceServer m_srv_generate_training_data_from_random;
	ros::ServiceClient m_srv_client_get_path_costs;

	DataSerializer m_data_serializer;

	float m_map_size;
	float m_resolution_detailed;
	float m_resolution_abstract;
	int m_map_size_cells;
	int m_num_orientations;
	int m_num_orientations_abstract;

	int m_num_random_maps_1_obstacle;
	int m_num_random_maps_2_obstacle;
	int m_num_random_maps_3_obstacle;
	int m_num_random_maps_1_wall;
	int m_num_random_maps_2_wall;
	int m_num_random_maps_1_wall_1_obstacle;
	int m_num_random_maps_stairs;
	int m_num_random_maps_stairs_focussed;
	int m_num_random_maps_1_wall_stairs;

	std::string m_file_path;
};
}

#endif // TRAINING_DATA_GENERATOR_H
