import math
import numpy
import time
import os, os.path
from torch.utils.data import Dataset, DataLoader


class AbstractionLearningDataSet(Dataset):

	def StringToFloatList(self, input_string):
		output_list = []
		tmp_character_list = []

		for character in input_string:
			if character == " ":
				tmp_string = "".join(tmp_character_list)
				output_list.append(float(tmp_string))
				tmp_character_list = []
			else:
				tmp_character_list.append(character)

		return output_list


	def FloatListToMatrix(self, input_float_list):
		matrix_size = math.sqrt(len(input_float_list))

		if (matrix_size.is_integer()):
			matrix_size = int(matrix_size)
			matrix =  numpy.asarray(input_float_list)
			matrix = numpy.reshape(matrix, (matrix_size, matrix_size))
			return matrix
		else:
			print "FloatListToMatrix: input cannot be transformed to a square shaped matrix"


	def GetFeasibilityValue(self, input):
		return_list = []

		if input > 1000.0:
			return_list.append(float(0.0))
			return numpy.asarray(return_list)
		else:
			return_list.append(float(1.0))
			return numpy.asarray(return_list)


	def ReadTrainingData(self):
		self.train_data = []

		file_object = open(self.file_path, "r")

		max_line_number = 5
		line_number = 1
		scene = {"heights": [], "goal_poses": [], "costs": [], "feasible": []}

		goal_pose_array = []
		cost_array = []
		feasible_array = []

		while line_number <= max_line_number:
			line = file_object.readline()

			if line_number == 4:
				num_maps = int(line)
				max_line_number = 5 + num_maps * 25

			if line_number > 6:
				if (line_number - 6) % 25 == 1:
					heights = self.FloatListToMatrix(self.StringToFloatList(line))
					scene["heights"] = heights
				if (line_number - 6) % 25 >= 2 and (line_number - 6) % 25 < 24:
					task = self.StringToFloatList(line)
					goal_pose = task[0:3]
					tmp_cost = task[3]
					feasible = self.GetFeasibilityValue(tmp_cost)
					cost = []
					cost.append(tmp_cost)
					cost = numpy.asarray(cost)
					goal_pose_array.append(goal_pose)
					cost_array.append(cost)
					feasible_array.append(feasible)
				if (line_number - 6) % 25 == 24:
					scene["goal_poses"] = goal_pose_array
					scene["costs"] = cost_array
					scene["feasible"] = feasible_array
					self.train_data.append(scene)
					goal_pose_array = []
					cost_array = []
					feasible_array = []
					scene = {"heights": [], "goal_poses": [], "costs": [], "feasible": []}#

			line_number += 1
		file_object.close()


	def __init__(self, file_path):
		t_start = time.time()
		self.file_path = file_path
		self.ReadTrainingData()


	def __len__(self):
		t_start = time.time()
		length = len(self.train_data) * 22
		return length


	def __getitem__(self, idx):
		t_start = time.time()
		map_index = idx//22
		scene_index = idx%22

		train_scene = {"heights": [], "goal_pose": [], "costs": [], "feasible": []}
		train_scene["heights"] = self.train_data[map_index]["heights"]
		train_scene["goal_pose"] = numpy.asarray(self.train_data[map_index]["goal_poses"][scene_index])
		train_scene["costs"] = self.train_data[map_index]["costs"][scene_index]
		train_scene["feasible"] = self.train_data[map_index]["feasible"][scene_index]
		return train_scene
