// Struct which describes robot poses
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2016

#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sstream>
#include <string>

namespace planning_abstraction_net
{
  
struct RobotPose
{
	// robot base coordinates in map ccordinates 
	Eigen::Vector2f base_coordinate;

	// robot base roatation around the z-axis relativ to the map coordinate system (yaw angle)
	float orientation;

	// ground clearance of the robot base
	float clearance;

	// wheel coordinates relativ to the robot base in robot coordinates
	Eigen::Vector2f rel_wheel_coordinate_fl;
	Eigen::Vector2f rel_wheel_coordinate_bl;
	Eigen::Vector2f rel_wheel_coordinate_br;
	Eigen::Vector2f rel_wheel_coordinate_fr;
	
	// wheel coordinates in absolute coordinates
	Eigen::Vector2f abs_wheel_coordinate_fl;
	Eigen::Vector2f abs_wheel_coordinate_bl;
	Eigen::Vector2f abs_wheel_coordinate_br;
	Eigen::Vector2f abs_wheel_coordinate_fr;

	float leg_height_fl = -1.0f;
	float leg_height_bl = -1.0f;
	float leg_height_br = -1.0f;
	float leg_height_fr = -1.0f;
	
	// inform about base shift
	bool base_shift_required = false;

	// information if a step is required to get from the last pose to this pose and if this step is evaluated already
	bool step_required = false;
	bool evaluation_required = false;
	float step_length = 0.0f;
	float step_max_rel_height = 0.0f;
	float step_rel_end_height = 0.0f;

	// information about base roll
	bool roll_required = false;
	float y_com = 0.0f;
	
	// pitch information
	bool pitch_required = false;
	float pitch_angle = 0.0f;
	
	// careful driving flag
	bool careful_driving = false;
	
	// representation level
	int representation_level = -1;
  

};


inline void FitOrientationToGrid (float& orientation, const int num_orientations)
{
	float orientation_step = 2 * M_PI / num_orientations;
	if (orientation < 0)
		orientation += 2 * M_PI;
	
	orientation = fmod(std::floor((orientation / orientation_step) + 0.5), num_orientations) * orientation_step;
}


inline float GetRobotOrientationFromAngle (const float angle)
{
	float result = std::fmod(angle + 1e-3, 2 * M_PI) - 1e-3;
	if (result < 0)
	{
		if (result > -1e-3)
			result = 0.0f;
		else
			result += 2 * M_PI;
	}
	return result;
}


inline float GetDifferenceBetweenTwoOrientations(const float orientation1, const float orientation2)
{
	return std::min(std::fabs(orientation1 - orientation2), 2.0f * (float)M_PI - std::fabs(orientation1 - orientation2));
}


inline void FillAbsoluteWheelCoordinatesFL (RobotPose& pose)
{
	Eigen::Rotation2D<float> rotation2D(pose.orientation);
	pose.abs_wheel_coordinate_fl = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_fl.x(), pose.rel_wheel_coordinate_fl.y() - pose.y_com);
}


inline void FillAbsoluteWheelCoordinatesBL (RobotPose& pose)
{
	Eigen::Rotation2D<float> rotation2D(pose.orientation);
	pose.abs_wheel_coordinate_bl = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_bl.x(), pose.rel_wheel_coordinate_bl.y() - pose.y_com);
}


inline void FillAbsoluteWheelCoordinatesBR (RobotPose& pose)
{
	Eigen::Rotation2D<float> rotation2D(pose.orientation);
	pose.abs_wheel_coordinate_br = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_br.x(), pose.rel_wheel_coordinate_br.y() - pose.y_com);
}


inline void FillAbsoluteWheelCoordinatesFR (RobotPose& pose)
{
	Eigen::Rotation2D<float> rotation2D(pose.orientation);
	pose.abs_wheel_coordinate_fr = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_fr.x(), pose.rel_wheel_coordinate_fr.y() - pose.y_com);
}


inline void FillAbsoluteWheelCoordinates (RobotPose& pose)
{
	Eigen::Rotation2D<float> rotation2D(pose.orientation);
	if (pose.representation_level == 1)
	{
		pose.abs_wheel_coordinate_fl = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_fl.x(), pose.rel_wheel_coordinate_fl.y() - pose.y_com);
		pose.abs_wheel_coordinate_bl = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_bl.x(), pose.rel_wheel_coordinate_bl.y() - pose.y_com);
		pose.abs_wheel_coordinate_br = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_br.x(), pose.rel_wheel_coordinate_br.y() - pose.y_com);
		pose.abs_wheel_coordinate_fr = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_fr.x(), pose.rel_wheel_coordinate_fr.y() - pose.y_com);
	}
	else if (pose.representation_level == 2)
	{
		pose.abs_wheel_coordinate_fl = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_fl.x(), 0.0f - pose.y_com);
		pose.abs_wheel_coordinate_bl = pose.base_coordinate + rotation2D * Eigen::Vector2f(pose.rel_wheel_coordinate_bl.x(), 0.0f - pose.y_com);
		pose.abs_wheel_coordinate_br = Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
		pose.abs_wheel_coordinate_fr = Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
	}
	else
	{
		pose.abs_wheel_coordinate_fl = Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
		pose.abs_wheel_coordinate_bl = Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
		pose.abs_wheel_coordinate_br = Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
		pose.abs_wheel_coordinate_fr = Eigen::Vector2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
	}
}

inline std::string PrintRobotPose (const RobotPose pose)
{
	std::ostringstream result;
	result << "(" << pose.base_coordinate.x() << "," << pose.base_coordinate.y() << "/ " << pose.orientation << ")";
	return result.str();
}

inline std::string PrintRobotPoseDetailed (const RobotPose pose)
{
	std::ostringstream result;
	result << "(" << pose.base_coordinate.x() << "," << pose.base_coordinate.y() << "/ " << pose.orientation 
	       << "), rel. wheel x-positions fl/bl/br/fr: " << pose.rel_wheel_coordinate_fl.x() << "/" << pose.rel_wheel_coordinate_fl.y() << "," 
						            << pose.rel_wheel_coordinate_bl.x() << "/" << pose.rel_wheel_coordinate_bl.y() << "," 
							    << pose.rel_wheel_coordinate_br.x() << "/" << pose.rel_wheel_coordinate_br.y() << "," 
							    << pose.rel_wheel_coordinate_fr.x() << "/" << pose.rel_wheel_coordinate_fr.y() 
	       << ", SR: " << pose.step_required 
	       << ", ER: " << pose.evaluation_required
	       << ", BSR: " << pose.base_shift_required
	       << ", RR: " << pose.roll_required
	       << ", y_com: " << pose.y_com
	       << ", RL: " << pose.representation_level;
	return result.str();
}

inline std::string PrintRobotPoseWithHeights (const RobotPose pose)
{
	std::ostringstream result;
	result << "(" << pose.base_coordinate.x() << "," << pose.base_coordinate.y() << "/ " << pose.orientation 
	<< "), rel. wheel x-positions and heights: fl: " << pose.rel_wheel_coordinate_fl.x() << "/" << pose.leg_height_fl
	                                     << ", bl: " << pose.rel_wheel_coordinate_bl.x() << "/" << pose.leg_height_bl
					     << ", br: " << pose.rel_wheel_coordinate_br.x() << "/" << pose.leg_height_br
					     << ", fr: " << pose.rel_wheel_coordinate_fr.x() << "/" << pose.leg_height_fr
					     << ", SR: " << pose.step_required
					     << ", ER: " << pose.evaluation_required
					     << ", BSR: " << pose.base_shift_required
					     << ", RR: " << pose.roll_required
					     << ", PR: " << pose.pitch_required
					     << ", y_com: " << pose.y_com
					     << ", pa: " << pose.pitch_angle;
	return result.str();
}


inline bool operator== (const RobotPose& rp1, const RobotPose& rp2) 
{  
	if ((std::fabs(rp1.rel_wheel_coordinate_bl.x() - rp2.rel_wheel_coordinate_bl.x()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_bl.x()) && std::isnan(rp2.rel_wheel_coordinate_bl.x())))
	 && (std::fabs(rp1.rel_wheel_coordinate_br.x() - rp2.rel_wheel_coordinate_br.x()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_br.x()) && std::isnan(rp2.rel_wheel_coordinate_br.x())))
	 && (std::fabs(rp1.rel_wheel_coordinate_fr.x() - rp2.rel_wheel_coordinate_fr.x()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_fr.x()) && std::isnan(rp2.rel_wheel_coordinate_fr.x())))
	 && (std::fabs(rp1.rel_wheel_coordinate_fl.x() - rp2.rel_wheel_coordinate_fl.x()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_fl.x()) && std::isnan(rp2.rel_wheel_coordinate_fl.x())))
	 && std::fabs(rp1.base_coordinate.x() - rp2.base_coordinate.x()) < 1e-3
	 && std::fabs(rp1.base_coordinate.y() - rp2.base_coordinate.y()) < 1e-3
	 && (std::fabs(rp1.orientation - rp2.orientation) < 1e-3 || (std::isnan(rp1.orientation) && std::isnan(rp2.orientation)))
	 && (std::fabs(rp1.rel_wheel_coordinate_fl.y() - rp2.rel_wheel_coordinate_fl.y()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_fl.y()) && std::isnan(rp2.rel_wheel_coordinate_fl.y())))
	 && (std::fabs(rp1.rel_wheel_coordinate_bl.y() - rp2.rel_wheel_coordinate_bl.y()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_bl.y()) && std::isnan(rp2.rel_wheel_coordinate_bl.y())))
	 && (std::fabs(rp1.rel_wheel_coordinate_br.y() - rp2.rel_wheel_coordinate_br.y()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_br.y()) && std::isnan(rp2.rel_wheel_coordinate_br.y())))
	 && (std::fabs(rp1.rel_wheel_coordinate_fr.y() - rp2.rel_wheel_coordinate_fr.y()) < 1e-3 || (std::isnan(rp1.rel_wheel_coordinate_fr.y()) && std::isnan(rp2.rel_wheel_coordinate_fr.y()))))
		return true;

	return false;
}


}

#endif // ROBOT_POSE_H
