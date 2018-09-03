// Parameters for Centauro Planner
// Author: Tobias Klamt <klamt@ais.uni-bonn.de
// 2016

#include <planning_abstraction_net/priority_queue_discr_pose.h>


namespace planning_abstraction_net
{
typedef fibonacci_heap<std::pair<Eigen::Vector3i, float>, compare<heap_compare_discr_pose>> fib_heap;

void PriorityQueueDiscrPose::Clear()
{
	m_priority_queue = fib_heap();
}


fib_heap::handle_type PriorityQueueDiscrPose::push (const Eigen::Vector3i cell, const float f_cost)
{
	return m_priority_queue.push(std::pair<Eigen::Vector3i, float>(cell, f_cost));
}


std::pair<Eigen::Vector3i, float> PriorityQueueDiscrPose::top()
{
	return m_priority_queue.top();
}


void PriorityQueueDiscrPose::pop()
{
	m_priority_queue.pop();
}


void PriorityQueueDiscrPose::DecreaseKey(const Eigen::Vector3i& cell, const float new_f_cost, fib_heap::handle_type priority_queue_handle)
{
	m_priority_queue.increase(priority_queue_handle, std::pair<Eigen::Vector3i, float>(cell, new_f_cost));
}


unsigned int PriorityQueueDiscrPose::size()
{
	return m_priority_queue.size();
}
}
