// Serialize abstraction learning data to access it in python
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#include <planning_abstraction_net/data_serializer.h>

namespace planning_abstraction_net
{
DataSerializer::DataSerializer()
{
}


void DataSerializer::SerializeData (const std::vector<cv::Mat_<float>>& map_crops,
				    const std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& poses,
				    const std::vector<std::vector<float>>& costs,
				    const std::string file_path)
{
	std::ofstream file;
	file.open(file_path);
	file << "Abstraction learning data\n";
	file << "Each paragraph describes one scene: Line 1: map counter | line 2: map | line 3-21: goal pose with correspondent costs (x, y, yaw, cost) \n";
	file << "Num maps:\n" << std::to_string(map_crops.size()) << "\n";

	for (uint map_counter = 0; map_counter < map_crops.size(); map_counter++)
	{
		// add map crop
		cv::Mat_<float> map_crop = map_crops[map_counter];
		file << "\n" << std::to_string(map_counter) << "\n";
		std::ostringstream map_crop_string;
		for (float map_crop_element : map_crop)
			map_crop_string << std::to_string(map_crop_element) << " ";
		file << map_crop_string.str() << "\n";

		// add goal poses and respective costs
		for (uint task_counter = 0; task_counter < poses[map_counter].size(); task_counter++)
		{
			RobotPose goal_pose = poses[map_counter][task_counter].second;
			float cost = costs[map_counter][task_counter];
			file << std::to_string(goal_pose.base_coordinate.x()) << " " << std::to_string(goal_pose.base_coordinate.y()) << " " << std::to_string(goal_pose.orientation) << " " << std::to_string(cost) << " " << "\n";

		}
	}
	file.close();
}
}
