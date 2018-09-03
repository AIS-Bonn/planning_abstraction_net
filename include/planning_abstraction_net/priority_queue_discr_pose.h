// This is a second implementation of a priority queue which uses a heap
// Author: Tobias Klamt <klamt@ais.uni-bonn.de>
// 2016

#ifndef PRIORITY_QUEUE_DISCR_POSE_H
#define PRIORITY_QUEUE_DISCR_POSE_H

#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/policies.hpp>

#include <Eigen/Core>


using namespace boost::heap;

namespace planning_abstraction_net
{

struct heap_compare_discr_pose
{
	bool operator()(const std::pair<Eigen::Vector3i, float>& p1, const std::pair<Eigen::Vector3i, float>& p2) const
	{
		if (!std::isnan(p1.second) && !std::isnan(p2.second))
			return p1.second > p2.second;
		if (!std::isnan(p1.second))
			return false;
		return true;
	}
};


class PriorityQueueDiscrPose
{
typedef fibonacci_heap<std::pair<Eigen::Vector3i, float>, compare<heap_compare_discr_pose>> fib_heap;
public:
	void Clear ();
	
	fib_heap::handle_type push (const Eigen::Vector3i cell, const float f_cost);
		
	std::pair<Eigen::Vector3i, float> top ();
		
	void pop ();
		
	void DecreaseKey (const Eigen::Vector3i& cell, const float new_f_cost, fib_heap::handle_type priority_queue_handle);
	
	unsigned int size ();
	
	
private:
	fib_heap m_priority_queue;
};
}


#endif // PRIORITY_QUEUE_CELL_H
