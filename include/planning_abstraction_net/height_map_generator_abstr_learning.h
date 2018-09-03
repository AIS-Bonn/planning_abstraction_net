// Generator for squared height map patches to train abstraction methods
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#ifndef HEIGHT_MAP_GENERATOR_ABSTR_LEARNING_H
#define HEIGHT_MAP_GENERATOR_ABSTR_LEARNING_H

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <ros/init.h>

namespace planning_abstraction_net
{

class HeightMapGeneratorAbstrLearning
{
public:
	HeightMapGeneratorAbstrLearning(const float map_size, const float resolution);

	void AddGroundPlane();

	void AddCuboid (const float size_x, const float size_y, const float size_z,
			const float pos_x, const float pos_y, const float pos_z,
			const float rot_x, const float rot_y, const float rot_z);

	void AddRandomCuboid(const float min_size, const float max_size,
			     const float min_height, const float max_height);

	void AddRandomWall(const float min_length, const float max_length,
			   const float min_thickness, const float max_thickness,
			   const float min_height, const float max_height);

	void AddRandomStair(const int min_number_of_steps, const int max_number_of_steps,
			    const float min_step_height, const float max_step_height,
			    const float min_step_length, const float max_step_length,
			    const float min_width, const float max_width,
			    const float min_orientation, const float max_orientation);

	cv::Mat_<float> GetHeightMap() const;

private:
	float m_map_size;
	float m_resolution;

	int m_num_cells;

	cv::Mat_<float> m_height_map;
};
}


#endif // HEIGHT_MAP_GENERATOR_ABSTR_LEARNING_H
