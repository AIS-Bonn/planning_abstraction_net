import torch
from torch.autograd import Variable
import torch.nn as nn
import torch.nn.functional as F

class AbstractionLearningCNN(nn.Module):
	def __init__(self):
		super(AbstractionLearningCNN, self).__init__()
		self.conv1 = nn.Conv2d(1, 3, kernel_size=(3,3), stride=1, padding=1)    # 72 * 72 @ 3
		self.conv2 = nn.Conv2d(3, 5, kernel_size=(7,7), stride=1, padding=3)    # 72 * 72 @ 5
		self.conv3 = nn.Conv2d(5, 28, kernel_size=(14,14), stride=1, padding=0) # 59 * 59 @ 28
		self.conv4 = nn.Conv2d(28, 31, kernel_size=(4,4), stride=1, padding=0)  # 56 * 56 @ 31 -> 28 * 28 @ 31
		self.conv5 = nn.Conv2d(31, 34, 3)  					# 26 * 26 @ 34 -> 13 * 13 @ 34
		self.conv6 = nn.Conv2d(34, 36, 3)  					# 11 * 11 @ 36
		self.conv7 = nn.Conv2d(36, 38, 3)  					# 9 * 9 @ 38
		self.conv8 = nn.Conv2d(38, 40, 3)  					# 7 * 7 @ 40

		self.pool = nn.MaxPool2d(2,2)

		self.fc1 = nn.Linear(7 * 7 * 40 + 3, 500)
		self.fc2 = nn.Linear(500, 150)
		self.fc3 = nn.Linear(150, 50)
		self.fc4 = nn.Linear(50, 20)
		self.fc_final_costs = nn.Linear(20, 1)
		self.fc_final_feasible = nn.Linear(20, 1)


	def forward(self, heights, goal_pose):
		# Convolutional layers
		x = F.relu(self.conv1(heights))  # 72 x 72 @ 3
		x = F.relu(self.conv2(x))        # 72 x 72 @ 5
		x = F.relu(self.conv3(x))        # 59 x 59 @ 8
		x = F.relu(self.conv4(x))        # 56 x 56 @ 31
		x = self.pool(x)                 # 28 x 28 @ 31
		x = F.relu(self.conv5(x))        # 26 x 26 @ 34
		x = self.pool(x)                 # 13 x 13 @ 34
		x = F.relu(self.conv6(x))        # 11 x 11 @ 36
		x = F.relu(self.conv7(x))        # 9 x 9 @ 38
		x = F.relu(self.conv8(x))        # 7 x 7 @ 40

		# Reshape to vector
		x = x.view(-1, 7 * 7 * 40)       # 1440

		# Concatenate with goal pose
		y = torch.cat((x, goal_pose), 1) # 1443

		# Fully connected layers
		y = F.relu(self.fc1(y))          # 500
		y = F.relu(self.fc2(y))          # 150
		y = F.relu(self.fc3(y))          # 50
		y = F.relu(self.fc4(y))          # 20
		y_costs = self.fc_final_costs(y) # 1
		y_feasible = torch.sigmoid(self.fc_final_feasible(y)) # 1
		return y_costs, y_feasible



