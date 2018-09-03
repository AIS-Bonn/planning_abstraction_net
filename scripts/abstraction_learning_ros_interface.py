#!/usr/bin/env python

import math
import rospy
import std_srvs.srv
import time
import torch
from torch.autograd import Variable
from planning_abstraction_net.srv import *
from abstraction_learning_CNN import AbstractionLearningCNN

use_GPU = torch.cuda.is_available()

class AbstractionLearningRosInterface():
	def __init__(self):
		self.map_crops = torch.FloatTensor(1,1,1,1)
		self.net = AbstractionLearningCNN()

		if use_GPU:
			self.net = self.net.cuda()


	def handle_load_network(self, req):
		name = "CNN_checkpoint1.model"
		checkpoint = torch.load("../CNN_checkpoints/" + name)
		self.net.load_state_dict(checkpoint["state"])
		print"   * Network loaded *"
		return []


	def handle_set_map_crops(self, req):
		num_crops_x = req.num_patches_x
		num_crops_y = req.num_patches_y
		num_crops_orient = req.num_patches_orient
		crop_size = req.patch_size

		if list(self.map_crops.shape)[0] < 2:
			self.map_crops = torch.FloatTensor(num_patches_x * num_patches_y * num_patches_orient, 1, patch_size, patch_size)

		x_coordinates = torch.tensor(req.patch_x)
		y_coordinates = torch.tensor(req.patch_y)
		orientations = torch.tensor(req.patch_orient)
		heights = torch.tensor(req.heights)

		for i in range(len(x_coordinates)):
			x = x_coordinates[i]
			y = y_coordinates[i]
			orient = orientations[i]
			map_crop = heights[i * crop_size * crop_size : (i+1) * crop_size * crop_size].view(crop_size, crop_size)
			self.map_crops[x + y * num_crops_x + orient * num_crops_x * num_crops_y][0] = map_crop

		return SetMapCropsResponse()


	def handle_batch_abstraction(self, req):
		start_time = time.time()

		## Initialize ##
		request_batch_size = req.batch_size
		network_batch_size = 100	# This can be adjusted according to the available GPU memory
		num_network_iterations = (int)(req.batch_size/network_batch_size) + 1

		map_indices = torch.tensor(req.map_indices)
		goals_vector = torch.cat((torch.tensor(req.goals_x).view(-1,1), torch.tensor(req.goals_y).view(-1,1), torch.tensor(req.goals_orient).view(-1,1)), 1)

		feasible_vector = torch.FloatTensor
		cost_vector = torch.FloatTensor

		## The following three lines can uncommented if you have large GPU memory -> Further acceleration ##
		#if use_GPU:
			#self.map_crops = self.map_crops.cuda()
			#goals_vector = goals_vector.cuda()


		## Iterate through all full batches (the last batch might be shorter) ##
		for iteration in range(num_network_iterations-1):
			iteration_start_time = time.time()

			# Prepare data #
			network_batch_indices = range(iteration * network_batch_size, (iteration+1) * network_batch_size)
			tmp_map_indices = map_indices[network_batch_indices]
			heights_4d = self.height_map_patches[tmp_map_indices]
			goal_poses = goals_vector[network_batch_indices]

			heights_4d = Variable(heights_4d)
			goal_poses = Variable(goal_poses)

			if use_GPU:
				heights_4d = heights_4d.cuda()
				goal_poses = goal_poses.cuda()

			# Process data in CNN #
			output_costs, output_feasible = self.net(heights_4d, goal_poses)

			# Save CNN output #
			if iteration == 0:
				feasible_vector = output_feasible.data
				cost_vector = output_costs.data
			else:
				feasible_vector = torch.cat((feasible_vector, output_feasible.data), 0)
				cost_vector = torch.cat((cost_vector, output_costs.data), 0)

			print("Iteration %6d of %6d of batch size %6d took %.3f seconds. The estimated overall time is %.3f seconds"
			%(iteration+1, num_network_iterations, network_batch_size, (time.time() - iteration_start_time), (num_network_iterations * (time.time() - iteration_start_time))))


		## Do the last batch ##
		iteration_start_time = time.time()

		# Prepare data #
		last_network_batch_size = request_batch_size - (num_network_iterations-1) * network_batch_size
		network_batch_indices = range((num_network_iterations-1) * network_batch_size, request_batch_size)
		tmp_map_indices = map_indices[network_batch_indices]
		heights_4d = self.height_map_patches[tmp_map_indices]
		goal_poses = goals_vector[network_batch_indices]

		heights_4d = Variable(heights_4d)
		goal_poses = Variable(goal_poses)

		if use_GPU:
			heights_4d = heights_4d.cuda()
			goal_poses = goal_poses.cuda()

		# Process data in CNN #
		output_costs, output_feasible = self.net(heights_4d, goal_poses)

		# Save CNN output #
		feasible_vector = torch.cat((feasible_vector, output_feasible.data), 0)
		cost_vector = torch.cat((cost_vector, output_costs.data), 0)

		print("Iteration %6d of %6d of batch size %6d took %.3f seconds. The estimated overall time is %.3f seconds"
		%(num_network_iterations, num_network_iterations, last_network_batch_size, (time.time() - iteration_start_time), (num_network_iterations * (time.time() - iteration_start_time))))



		## Generate result vector ##
		cost_vector[feasible_vector < 0.5] = float('Inf')
		print("The whole batch abstraction took %.3f seconds" %(time.time() - start_time))

		self.height_map_patches=torch.FloatTensor(1,1,1,1)
		return GetBatchAbstractionResponse(cost_vector)



	def abstraction_learning_network_service_server(self):
		rospy.init_node("abstraction_learning_network_service_server")
		l = rospy.Service("abstraction_learning_network/load_network", std_srvs.srv.Empty(), self.handle_load_network)
		l = rospy.Service("abstraction_learning_network/set_map_crops", SetMapCrops, self.handle_set_map_crops)
		m = rospy.Service("abstraction_learning_network/get_batch_abstraction", GetBatchAbstraction, self.handle_batch_abstraction)
		print "   ***   ROS Interface for Abstraction Learning Network is running   ***   "
		rospy.spin()


if __name__ == "__main__":
	interface = AbstractionLearningRosInterface()
	interface.abstraction_learning_network_service_server()




