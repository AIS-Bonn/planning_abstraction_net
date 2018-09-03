import time
import torch
from abstraction_learning_dataset import AbstractionLearningDataSet
from abstraction_learning_CNN import AbstractionLearningCNN
from torch.autograd import Variable
import torch.nn as nn
import torch.optim as optim


def TrainNetwork():
	use_GPU = torch.cuda.is_available()
	print ("Train Abstraction Learning CNN, use GPU: ", use_GPU)

	## Define training data set and test data sets ##
	time_before_reading = time.time()
	# This needs to be replaced by the training data set which can be generated with the dataset_generator
	trainset = AbstractionLearningDataSet(file_path="../data_sets/random_test_data_set.txt")
	trainloader = torch.utils.data.DataLoader(trainset, batch_size=1, shuffle=True, num_workers=4)
	print ("Read Training       Dataset which took %.3fs" %(time.time() - time_before_reading))
	time_before_reading = time.time()

	testset_1_random = AbstractionLearningDataSet(file_path="../data_sets/random_test_data_set.txt")
	testloader_1_random = torch.utils.data.DataLoader(testset_1_random, batch_size=1,shuffle=False, num_workers=4)
	print ("Read Random Test    Dataset which took %.3fs" %(time.time() - time_before_reading))
	time_before_reading = time.time()

	testset_2_simulated = AbstractionLearningDataSet(file_path="../data_sets/simulated_test_data_set.txt")
	testloader_2_simulated = torch.utils.data.DataLoader(testset_2_simulated, batch_size=1,shuffle=False, num_workers=4)
	print ("Read Simulated Test Dataset which took %.3fs" %(time.time() - time_before_reading))
	time_before_reading = time.time()

	testset_3_real = AbstractionLearningDataSet(file_path="../data_sets/real_world_test_data_set.txt")
	testloader_3_real = torch.utils.data.DataLoader(testset_3_real, batch_size=1,shuffle=False, num_workers=4)
	print ("Read Real Test      Dataset which took %.3fs" %(time.time() - time_before_reading))

	trainset_size = len(trainset)
	testset_rand_size = len(testset_1_random)
	testset_simu_size = len(testset_2_simulated)
	testset_real_size = len(testset_3_real)


	## CNN initialization ##
	# Net
	name = "AbstractionLearningCNN"
	net = AbstractionLearningCNN()
	if use_GPU:
		net = net.cuda()
	print(net)

	# Loss function and optimizer
	learning_rate = 0.0001
	costs_loss_function = nn.L1Loss()
	feasibility_loss_function = nn.BCELoss()
	optimizer = optim.SGD(net.parameters(), learning_rate, momentum=0.9)
	cost_loss_weight = float(1.0)
	feasible_loss_weight = float(1.0)
	print("Learning rate = ", learning_rate)


	## Train the CNN ##
	print("\nStart training")
	last_cost_loss = float('Inf')
	last_last_cost_loss = float('Inf')
	last_feasible_loss = float('Inf')
	last_last_feasible_loss = float('Inf')

	for epoch in range(100):
		start_time = time.time()

		# set all losses to zero
		running_cost_loss = 0.0
		running_feasible_loss = 0.0
		cost_loss_counter = 0
		feasible_loss_counter = 0

		for i, sample in enumerate(trainloader, 0):
			# get the data
			heights = sample["heights"].float()
			goal_pose = sample["goal_pose"].float()
			costs = sample["costs"].float()
			feasible = sample["feasible"].float()

			# discretize goal pose
			goal_pose[0][0] = float(int(goal_pose[0][0]/0.1))
			goal_pose[0][1] = float(int(goal_pose[0][1]/0.1))
			if goal_pose[0][2] > 0.01:
				goal_pose[0][2] = 1.0
			if goal_pose[0][2] < -0.01:
				goal_pose[0][2] = -1.0

			# add 1 dimension to height input
			heights_4d = torch.FloatTensor(1,1,72,72)
			heights_4d[0] = heights

			# send them to GPU if available
			if use_GPU:
				heights_4d = heights_4d.cuda()
				goal_pose = goal_pose.cuda()
				costs = costs.cuda()
				feasible = feasible.cuda()

			# wrap them in Variable
			heights_4d = Variable(heights_4d)
			goal_pose = Variable(goal_pose)
			costs = Variable(costs)
			feasible = Variable(feasible)

			# zero the parameter gradients
			optimizer.zero_grad()

			# forward + backward + optimize
			output_costs, output_feasible = net(heights_4d, goal_pose)
			costs_loss = costs_loss_function(output_costs, costs)
			feasible_loss = feasibility_loss_function(output_feasible, feasible)

			if (feasible.data[0][0] > 0.5):
				loss = cost_loss_weight * costs_loss + feasible_loss_weight * feasible_loss
				running_cost_loss += costs_loss.data
				running_feasible_loss += feasible_loss.data
				cost_loss_counter += 1
				feasible_loss_counter += 1
			else:
				loss = feasible_loss_weight * feasible_loss
				running_feasible_loss += feasible_loss.data
				feasible_loss_counter += 1

			if use_GPU:
				loss = loss.cuda()

			loss.backward()
			optimizer.step()


		current_feasible_loss = running_feasible_loss.item() / max(feasible_loss_counter, 1)
		current_cost_loss = running_cost_loss.item() / max(cost_loss_counter, 1)

		training_time = time.time() - start_time

		# Save the CNN at the current episode
		checkpoint = {
			"name": name,
			"state": net.state_dict(),
			"epoch": epoch,
		}
		torch.save(checkpoint, "../CNN_checkpoints/{}_epoch_{}_lr{}.model".format(name, epoch+1, learning_rate))




		## Evaluate the network on three different data sets ##
		## Random scenes ##
		rand_feasibility_correct = 0
		rand_sum_costs_at_feasible = 0.0
		rand_sum_costs_at_feasible_counter = 0
		rand_sum_cost_diffs_at_feasible = 0.0
		rand_start_time = time.time()

		for i, sample in enumerate(testloader_1_random, 0):

			# get the data
			heights = sample["heights"].float()
			goal_pose = sample["goal_pose"].float()
			costs = sample["costs"].float()
			feasible = sample["feasible"].float()

			# discretize goal pose
			goal_pose[0][0] = float(int(goal_pose[0][0]/0.1))
			goal_pose[0][1] = float(int(goal_pose[0][1]/0.1))
			if goal_pose[0][2] > 0.01:
				goal_pose[0][2] = 1.0
			if goal_pose[0][2] < -0.01:
				goal_pose[0][2] = -1.0

			# add 1 dimension to height input
			heights_4d = torch.FloatTensor(1,1,72,72)
			heights_4d[0] = heights

			# send them to GPU if available
			if use_GPU:
				heights_4d = heights_4d.cuda()
				goal_pose = goal_pose.cuda()

			# wrap them in Variable
			heights_4d = Variable(heights_4d)
			goal_pose = Variable(goal_pose)

			# get network results
			output_costs, output_feasible = net(heights_4d, goal_pose)

			if float(output_feasible.data[0]) > 0.5 and float(feasible[0]) > 0.5:
				rand_feasibility_correct += 1

				if float(feasible[0]) > 0.5:
					rand_sum_costs_at_feasible += float(output_costs.data[0])
					rand_sum_cost_diffs_at_feasible += abs(float(costs) - float(output_costs.data[0]))
					rand_sum_costs_at_feasible_counter += 1

			elif float(output_feasible.data[0]) < 0.5 and float(feasible[0]) < 0.5:
				rand_feasibility_correct += 1
		rand_eval_time = time.time() - rand_start_time

		## Simulated scenes ##
		simu_feasibility_correct = 0
		simu_sum_costs_at_feasible = 0.0
		simu_sum_costs_at_feasible_counter = 0
		simu_sum_cost_diffs_at_feasible = 0.0
		simu_start_time = time.time()

		for i, sample in enumerate(testloader_2_simulated, 0):
			# get the data
			heights = sample["heights"].float()
			goal_pose = sample["goal_pose"].float()
			costs = sample["costs"].float()
			feasible = sample["feasible"].float()

			# discretize goal pose
			goal_pose[0][0] = float(int(goal_pose[0][0]/0.1))
			goal_pose[0][1] = float(int(goal_pose[0][1]/0.1))
			if goal_pose[0][2] > 0.01:
				goal_pose[0][2] = 1.0
			if goal_pose[0][2] < -0.01:
				goal_pose[0][2] = -1.0

			# add 1 dimension to height input
			heights_4d = torch.FloatTensor(1,1,72,72)
			heights_4d[0] = heights

			# send them to GPU if available
			if use_GPU:
				heights_4d = heights_4d.cuda()
				goal_pose = goal_pose.cuda()

			# wrap them in Variable
			heights_4d = Variable(heights_4d)
			goal_pose = Variable(goal_pose)

			# get network results
			output_costs, output_feasible = net(heights_4d, goal_pose)

			if float(output_feasible.data[0]) > 0.5 and float(feasible[0]) > 0.5:
				simu_feasibility_correct += 1

				if float(feasible[0]) > 0.5:
					simu_sum_costs_at_feasible += float(output_costs.data[0])
					simu_sum_cost_diffs_at_feasible += abs(float(costs) - float(output_costs.data[0]))
					simu_sum_costs_at_feasible_counter += 1

			elif float(output_feasible.data[0]) < 0.5 and float(feasible[0]) < 0.5:
				simu_feasibility_correct += 1
		simu_eval_time = time.time() - simu_start_time



		## Real world scenes ##
		real_feasibility_correct = 0
		real_sum_costs_at_feasible = 0.0
		real_sum_costs_at_feasible_counter = 0
		real_sum_cost_diffs_at_feasible = 0.0
		real_start_time = time.time()

		for i, sample in enumerate(testloader_3_real, 0):
			# get the data
			heights = sample["heights"].float()
			goal_pose = sample["goal_pose"].float()
			costs = sample["costs"].float()
			feasible = sample["feasible"].float()

			# discretize goal pose
			goal_pose[0][0] = float(int(goal_pose[0][0]/0.1))
			goal_pose[0][1] = float(int(goal_pose[0][1]/0.1))
			if goal_pose[0][2] > 0.01:
				goal_pose[0][2] = 1.0
			if goal_pose[0][2] < -0.01:
				goal_pose[0][2] = -1.0

			# add 1 dimension to height input
			heights_4d = torch.FloatTensor(1,1,72,72)
			heights_4d[0] = heights

			# send them to GPU if available
			if use_GPU:
				heights_4d = heights_4d.cuda()
				goal_pose = goal_pose.cuda()

			# wrap them in Variable
			heights_4d = Variable(heights_4d)
			goal_pose = Variable(goal_pose)

			# get network results
			output_costs, output_feasible = net(heights_4d, goal_pose)

			if float(output_feasible.data[0]) > 0.5 and float(feasible[0]) > 0.5:
				real_feasibility_correct += 1

				if float(feasible[0]) > 0.5:
					real_sum_costs_at_feasible += float(output_costs.data[0])
					real_sum_cost_diffs_at_feasible += abs(float(costs) - float(output_costs.data[0]))
					real_sum_costs_at_feasible_counter += 1

			elif float(output_feasible.data[0]) < 0.5 and float(feasible[0]) < 0.5:
				real_feasibility_correct += 1

		real_eval_time = time.time() - real_start_time


		print("[%3d](%ds) F-Loss: %.4f, C-loss: %.4f for %d | RANDOM(%ds): F: %d/%d (%.1f%%), avg C: %.3f, avg. error: %.3f | SIMULATED(%ds): F: %d/%d (%.1f%%), avg C: %.3f, avg error: %.3f | REAL(%ds): F: %d/%d (%.1f%%), avg C: %.3f, avg error: %.3f"
		%(epoch + 1, training_time, running_feasible_loss / max(feasible_loss_counter,1), running_cost_loss / max(cost_loss_counter,1), cost_loss_counter,
		rand_eval_time, rand_feasibility_correct, testset_rand_size, 100 * rand_feasibility_correct/testset_rand_size, rand_sum_costs_at_feasible/max(rand_sum_costs_at_feasible_counter,1), rand_sum_cost_diffs_at_feasible/max(rand_sum_costs_at_feasible_counter,1),
		simu_eval_time, simu_feasibility_correct, testset_simu_size, 100 * simu_feasibility_correct/testset_simu_size, simu_sum_costs_at_feasible/max(simu_sum_costs_at_feasible_counter,1), simu_sum_cost_diffs_at_feasible/max(simu_sum_costs_at_feasible_counter,1),
		real_eval_time, real_feasibility_correct, testset_real_size, 100 * real_feasibility_correct/testset_real_size, real_sum_costs_at_feasible/max(real_sum_costs_at_feasible_counter,1), real_sum_cost_diffs_at_feasible/max(real_sum_costs_at_feasible_counter,1)))


		## Adjust loss weights ##
		if current_feasible_loss > last_feasible_loss and last_feasible_loss > last_last_feasible_loss:
			feasible_loss_weight = feasible_loss_weight / 5.0
			last_feasible_loss = float('Inf')
			last_last_feasible_loss = float('Inf')
			last_cost_loss = float('Inf')
			last_last_cost_loss = float('Inf')
			print("New feasible loss weight is %.6f" %(feasible_loss_weight))
		else:
			last_last_feasible_loss = last_feasible_loss
			last_feasible_loss = current_feasible_loss

		if current_cost_loss > last_cost_loss and last_cost_loss > last_last_cost_loss:
			cost_loss_weight = cost_loss_weight / 5.0
			last_cost_loss = float('Inf')
			last_last_cost_loss = float('Inf')
			print("New cost loss weight is %.6f" %(cost_loss_weight))
		else:
			last_last_cost_loss = last_cost_loss
			last_cost_loss = current_cost_loss



TrainNetwork()
