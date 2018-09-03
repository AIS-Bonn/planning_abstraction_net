// Serialize abstraction learning data to access it in python
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#ifndef DATA_SERIALIZER_H
#define DATA_SERIALIZER_H

#include <planning_abstraction_net/robot_pose.h>
#include <opencv2/opencv.hpp>
#include <fstream>

namespace planning_abstraction_net
{
class DataSerializer
{
public:
	DataSerializer();

	void SerializeData (const std::vector<cv::Mat_<float>>& map_crops,
			    const std::vector<std::vector<std::pair<RobotPose, RobotPose>>>& poses,
			    const std::vector<std::vector<float>>& costs,
			    const std::string file_path);
};
}


#endif // DATA_SERIALIZER_H
