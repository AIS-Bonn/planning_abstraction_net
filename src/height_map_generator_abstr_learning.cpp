// Generator for squared height map patches to train abstraction methods
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2018

#include <planning_abstraction_net/height_map_generator_abstr_learning.h>

namespace planning_abstraction_net
{

HeightMapGeneratorAbstrLearning::HeightMapGeneratorAbstrLearning(const float map_size, const float resolution)
{
	m_map_size = map_size;
	m_resolution = resolution;

	m_num_cells = (int)((m_map_size + 1e-3) / m_resolution);

	m_height_map = cv::Mat_<float>(m_num_cells, m_num_cells, std::numeric_limits<float>::quiet_NaN());
}


void HeightMapGeneratorAbstrLearning::AddGroundPlane()
{
	m_height_map = cv::Mat_<float>(m_num_cells, m_num_cells, 0.0f);
}


void HeightMapGeneratorAbstrLearning::AddCuboid (const float size_x, const float size_y, const float size_z,
						 const float pos_x, const float pos_y, const float pos_z,
						 const float rot_x, const float rot_y, const float rot_z)
{
	float cuboid_resolution = 0.5 * m_resolution;

	int steps_x = ceil(0.5 * size_x / cuboid_resolution);
	int steps_y = ceil(0.5 * size_y / cuboid_resolution);
	int steps_z = ceil(0.5 * size_z / cuboid_resolution);

	Eigen::AngleAxisf rollAngle(rot_x, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf pitchAngle(rot_y, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yawAngle(rot_z, Eigen::Vector3f::UnitZ());

	for(int x = -steps_x; x < steps_x + 1; ++x)
	{
		for(int y = -steps_y; y < steps_y + 1; ++y)
		{
			for(int z = -steps_z; z < steps_z + 1; ++z)
			{
				Eigen::Vector3f current_point = Eigen::Vector3f(x, y, z) * cuboid_resolution;
				current_point =  yawAngle * pitchAngle * rollAngle * current_point;
				current_point += Eigen::Vector3f(pos_x, pos_y, pos_z);

				if(current_point(0) < 0 || current_point(0) > m_map_size || current_point(1) < 0 || current_point(1) > m_map_size)
					continue;

				current_point(0) = std::floor(current_point(0) / m_resolution);
				current_point(1) = std::floor(current_point(1) / m_resolution);

				if (current_point(0) >= 0 && current_point(0) < m_num_cells
				 && current_point(1) >= 0 && current_point(1) < m_num_cells)
				{
					if (std::isnan(m_height_map(current_point(0), current_point(1)))
					 || m_height_map(current_point(0), current_point(1)) < current_point(2))
					{
						m_height_map(current_point(0), current_point(1)) = current_point(2);
					}
				}
			}
		}
	}
}


void HeightMapGeneratorAbstrLearning::AddRandomCuboid (const float min_size, const float max_size,
						       const float min_height, const float max_height)
{
	float size_x = min_size + (float)rand() / (float)RAND_MAX * (max_size - min_size);
	float size_y = min_size + (float)rand() / (float)RAND_MAX * (max_size - min_size);
	float size_z = min_height + (float)rand() / (float)RAND_MAX * (max_height - min_height);

	float pos_x = (float)rand() / (float)RAND_MAX * m_map_size;
	float pos_y = (float)rand() / (float)RAND_MAX * m_map_size;
	float pos_z = 0.5 * size_z;

	float rot_x = 0.0f;
	float rot_y = 0.0f;
	float rot_z = -M_PI + (float)rand() / (float)RAND_MAX * 2 * M_PI;

	AddCuboid(size_x, size_y, size_z, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z);
}


void HeightMapGeneratorAbstrLearning::AddRandomWall(const float min_length, const float max_length,
						    const float min_thickness, const float max_thickness,
						    const float min_height, const float max_height)
{
	float length = min_length + (float)rand() / (float)RAND_MAX * (max_length - min_length);
	float thickness = min_thickness + (float)rand() / (float)RAND_MAX * (max_thickness - min_thickness);
	float height = min_height + (float)rand() / (float)RAND_MAX * (max_height - min_height);

	float pos_x = (float)rand() / (float)RAND_MAX * m_map_size;
	float pos_y = (float)rand() / (float)RAND_MAX * m_map_size;
	float pos_z = 0.5f * height;

	float rot_x = 0.0f;
	float rot_y = 0.0f;
	float rot_z = -M_PI + (float)rand() / (float)RAND_MAX * 2 * M_PI;

	AddCuboid(length, thickness, height, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z);
}


void HeightMapGeneratorAbstrLearning::AddRandomStair(const int min_number_of_steps, const int max_number_of_steps,
						     const float min_step_height, const float max_step_height,
						     const float min_step_length, const float max_step_length,
						     const float min_width, const float max_width,
						     const float min_orientation, const float max_orientation)
{
	int num_steps = min_number_of_steps + rand() % (max_number_of_steps - min_number_of_steps + 1);
	float step_height = min_step_height + (float)rand() / (float)RAND_MAX * (max_step_height - min_step_height);
	float step_length = min_step_length + (float)rand() / (float)RAND_MAX * (max_step_length - min_step_length);
	float width = min_width + (float)rand() / (float)RAND_MAX * (max_width - min_width);

	// the position describes the position in the map where the first step starts
	float pos_x = (float)rand() / (float)RAND_MAX * m_map_size;
	float pos_y = (float)rand() / (float)RAND_MAX * m_map_size;

	float rot_x = 0.0f;
	float rot_y = 0.0f;
	float rot_z = min_orientation + (float)rand() / (float)RAND_MAX * (max_orientation - min_orientation);
	float cuboid_length = 2.0f;

	for (int step_number = 1; step_number <= num_steps; step_number++)
	{
		float cuboid_center_x = pos_x + cos(rot_z) * (0.5 * cuboid_length + (step_number - 1) * step_length);
		float cuboid_center_y = pos_y + sin(rot_z) * (0.5 * cuboid_length + (step_number - 1) * step_length);
		float cuboid_center_z = (0.5f + step_number - 1) * step_height;

		AddCuboid(cuboid_length, width, step_height, cuboid_center_x, cuboid_center_y, cuboid_center_z, rot_x, rot_y, rot_z);
	}
}


cv::Mat_<float> HeightMapGeneratorAbstrLearning::GetHeightMap() const
{
	return m_height_map;
}
}



