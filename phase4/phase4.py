# header files
import cv2
import numpy as np 
import math
import vrep
import time
import sys



# class for Astar
class Astar(object):
	# init function
	def __init__(self, start, goal, startOrientation, clearance, rpm1, rpm2, radius, wheelRadius, length, dt):
		self.start = start
		self.goal = goal
		self.k = startOrientation
		self.numRows = 1020
		self.numCols = 1020
		self.clearance = clearance
		self.rpm1 = rpm1
		self.rpm2 = rpm2
		self.radius = radius
		self.wheelRadius = wheelRadius
		self.length = length
		self.dt = dt
		self.goalbound=[]
		self.cost=np.full((int(self.numRows),int(self.numCols)), np.inf)
		self.euclidean_array=np.full((int(self.numRows),int(self.numCols)), np.inf)
		self.totalcost=np.full((int(self.numRows),int(self.numCols)), np.inf)
		self.visited=np.array(np.zeros((self.numRows,self.numCols)))
		self.parent_list=[]
		self.path=[]
	   
		
	# move is valid 
	def IsValid(self, currRow, currCol):
		if (currRow <= 1010 - (self.radius + self.clearance)) and (currRow >= 10 + (self.radius + self.clearance)) and (currCol <= 1010 - (self.radius + self.clearance)) and (currCol >= 10 + (self.radius + self.clearance)):             
			return 1
		else:
			return 0
	# checks for an obstacle
	def IsObstacle(self, row, col):
		safe_dist = self.clearance + self.radius
		# print(safe_dist,"safe_dist")
		circle1 = (row - 510)**2 + (col - 510)**2 - (100+safe_dist)**2 <= 0
		circle2 = (row - 710)**2 + (col - 210)**2 - (100+safe_dist)**2 <= 0
		circle3 = (row - 710)**2 + (col - 810)**2 - (100+safe_dist)**2 <= 0
		circle4 = (row - 310)**2 + (col - 810)**2 - (100+safe_dist)**2 <= 0
		square1 = row <= 385 + safe_dist and row >= 235 - safe_dist and col <= 285 + safe_dist and col >= 135 - safe_dist
		square2 = row <= 185 + safe_dist and row >= 35 -safe_dist and col <= 585 + safe_dist and col >= 435 -safe_dist
		square3 = row <= 985 + safe_dist and row >= 835 -safe_dist and col <= 585 + safe_dist and col >= 435 -safe_dist

		if (circle1 or circle2 or circle3 or circle4 or square1 or square2 or square3 ):
			output = 0
		else:
			output = 1
		
		return output

	# Calculates the movement for each action
	def Differential_motion(self,r1,r2,theta):
		thata = theta * ( 22 / ( 7 * 180 ) )
		dtheta=(self.wheelRadius*(r2-r1)*self.dt)/self.length + theta
		dtheta= dtheta * ( (7 * 180) / 22) 
		dtheta=dtheta%360
		if(dtheta == 360):
			dtheta = 0
		dx=(self.wheelRadius*(r1+r2)*math.cos(dtheta * (22 / (7 * 180)))*self.dt)/2
		dy=(self.wheelRadius*(r1+r2)*math.sin(dtheta * (22 / (7 * 180)))*self.dt)/2
		return dtheta,dx,dy


	# action move Straight
	def ActionStraight(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(r1,r1,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'Straight')
		return new_node


	# action move FastStraight
	def ActionFastStraight(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(r2,r2,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'FastStraight')
		return new_node


	# action move Left1
	def ActionLeft1(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(0,r1,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'Left1')
		return new_node
	
	
	# action move Left2
	def ActionLeft2(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(0,r2,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'Left2')
		return new_node
	

	# action move Left3
	def ActionLeft3(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(r1,r2,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'Left3')
		return new_node


	# action move Right1
	def ActionRight1(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(r1,0,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'Right1')
		return new_node


	# action move right2
	def ActionRight2(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(r2,0,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'Right2')
		return new_node


	# action move Right3
	def ActionRight3(self, currRow, currCol, currangle):
		newtheta, dx, dy = self.Differential_motion(r2,r1,currangle)
		new_node=(int(round(currRow+dx)),int(round(currCol+dy)),int(round(newtheta)),'Right3')
		return new_node


	#Checking if self.goal node reached or not
	def CheckIfGoal(self, currRow, currCol, angle):
		check = (((currRow - self.goal[0]) * (currRow - self.goal[0])) + ((currCol - self.goal[1]) * (currCol - self.goal[1])) - ( 2 * 2))
		if(check <= 0):

		
			print("goal reached")
			return True
		else:
			return False

	def Priority_Pop(self,Q):
		index_min=0
 
		X_min = Q[0][0] 
		Y_min = Q[0][1]
		for i in range(len(Q)):
			x = Q[i][0]
			y = Q[i][1]
			if self.totalcost[x,y] < self.totalcost[X_min,Y_min]:
				index_min = i
				X_min = x 
				Y_min= y
		currNode = Q[index_min]
		Q.remove(Q[index_min])
		return currNode

	def path_find(self,start):
			GN=self.goalbound
			# print('GN',GN)
			self.path.append(self.goalbound)
			while (GN!=start):
				z1 = GN[0]
				z2 = GN[1]
				a=self.parent_list[GN[0]][GN[1]]
				self.path.append(a)
				GN=a


	   
	# astar algorithm
	def Astar(self):
		# Converting RPM to radian per second
		global r1
		global r2
		r1 = self.rpm1*(math.pi/30)
		r2 = self.rpm2*(math.pi/30)
		print("Creating numpy arrays")
		for h in range (self.numRows):
			column=[]
			for g in range (self.numCols):
				column.append(0)
			self.parent_list.append(column)

		visited_node = []
		Q = []
		aa = self.start[0]
		bb = self.start[1]
		cc = self.k
		start1 = (aa,bb,self.k)
		Q.append([aa,bb,cc])
		self.cost[aa][bb]=0
		self.totalcost[aa][bb]=0
		currNode=[aa,bb,cc]

		while(len(Q) > 0):
			NoPath = 0
			if(self.CheckIfGoal(currNode[0],currNode[1],currNode[2]) == True):
				NoPath = 1
				self.goalbound=currNode
				break
			currNode=self.Priority_Pop(Q)
			straight_new = self.ActionStraight(currNode[0],currNode[1],currNode[2])
		
			status=self.IsObstacle(straight_new[0],straight_new[1])
			flag=self.IsValid(straight_new[0],straight_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[straight_new[0],straight_new[1]]==0:
					self.visited[straight_new[0],straight_new[1]]=1
					visited_node.append(straight_new)
					Q.append(straight_new)
					self.parent_list[straight_new[0]][straight_new[1]]=currNode
					self.cost[straight_new[0],straight_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
					self.euclidean_array[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-self.goal[0]),2) + math.pow((straight_new[1]-self.goal[1]),2))
					self.totalcost[straight_new[0],straight_new[1]]= self.cost[straight_new[0],straight_new[1]]+self.euclidean_array[straight_new[0],straight_new[1]]
				else:
					if self.cost[straight_new[0],straight_new[1]]>(self.cost[currNode[0],currNode[1]]+1):
						self.cost[straight_new[0],straight_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
						self.euclidean_array[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-self.goal[0]),2) + math.pow((straight_new[1]-self.goal[1]),2))
						self.parent_list[straight_new[0]][straight_new[1]]=currNode
						self.totalcost[straight_new[0],straight_new[1]]= self.cost[straight_new[0],straight_new[1]]+self.euclidean_array[straight_new[0],straight_new[1]]
	
	
			straight_faster_new=self.ActionFastStraight(currNode[0],currNode[1],currNode[2])
			status=self.IsObstacle(straight_faster_new[0],straight_faster_new[1])
			flag=self.IsValid(straight_faster_new[0],straight_faster_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[straight_faster_new[0],straight_faster_new[1]]==0:
					self.visited[straight_faster_new[0],straight_faster_new[1]]=1
					visited_node.append(straight_faster_new)
					Q.append(straight_faster_new)
					self.parent_list[straight_faster_new[0]][straight_faster_new[1]]=currNode
					self.cost[straight_faster_new[0],straight_faster_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
					self.euclidean_array[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-self.goal[0]),2) + math.pow((straight_faster_new[1]-self.goal[1]),2))
					self.totalcost[straight_faster_new[0],straight_faster_new[1]]= self.cost[straight_faster_new[0],straight_faster_new[1]]+self.euclidean_array[straight_faster_new[0],straight_faster_new[1]]
				else:
					if self.cost[straight_faster_new[0],straight_faster_new[1]]>(self.cost[currNode[0],currNode[1]]+1):
						self.cost[straight_faster_new[0],straight_faster_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
						self.euclidean_array[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-self.goal[0]),2) + math.pow((straight_faster_new[1]-self.goal[1]),2))
						self.parent_list[straight_faster_new[0]][straight_faster_new[1]]=currNode
						self.totalcost[straight_faster_new[0],straight_faster_new[1]]= self.cost[straight_faster_new[0],straight_faster_new[1]]+self.euclidean_array[straight_faster_new[0],straight_faster_new[1]]
	
			right_new=self.ActionRight1(currNode[0],currNode[1],currNode[2])
			status=self.IsObstacle(right_new[0],right_new[1])
			flag=self.IsValid(straight_new[0],straight_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[right_new[0],right_new[1]]==0:
					self.visited[right_new[0],right_new[1]]=1
					visited_node.append(right_new)
					Q.append(right_new)         
					self.parent_list[right_new[0]][right_new[1]]=currNode
					self.cost[right_new[0],right_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
					self.euclidean_array[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-self.goal[0]),2) + math.pow((right_new[1]-self.goal[1]),2))
					self.totalcost[right_new[0],right_new[1]]= self.cost[right_new[0],right_new[1]]+self.euclidean_array[right_new[0],right_new[1]]

				else:
					if self.cost[right_new[0],right_new[1]]>(self.cost[currNode[0],currNode[1]]+1):
						self.cost[right_new[0],right_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
						self.parent_list[right_new[0]][right_new[1]]=currNode
						self.euclidean_array[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-self.goal[0]),2) + math.pow((right_new[1]-self.goal[1]),2))
						self.totalcost[right_new[0],right_new[1]]= self.cost[right_new[0],right_new[1]]+self.euclidean_array[right_new[0],right_new[1]]
	
	
	
			left_new=self.ActionLeft1(currNode[0],currNode[1],currNode[2])
			status=self.IsObstacle(left_new[0],left_new[1])
			flag=self.IsValid(left_new[0],left_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[left_new[0],left_new[1]]==0:
					self.visited[left_new[0],left_new[1]]=1
					visited_node.append(left_new)
					Q.append(left_new)
					self.parent_list[left_new[0]][left_new[1]]=currNode
					self.euclidean_array[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-self.goal[0]),2) + math.pow((left_new[1]-self.goal[1]),2))
					self.cost[left_new[0],left_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
					self.totalcost[left_new[0],left_new[1]]= self.cost[left_new[0],left_new[1]]+self.euclidean_array[left_new[0],left_new[1]]
				else:
					if self.cost[left_new[0],left_new[1]]>(self.cost[currNode[0],currNode[1]]+1):
						self.cost[left_new[0],left_new[1]]=(self.cost[currNode[0],currNode[1]]+1)
						self.parent_list[left_new[0]][left_new[1]]=currNode
						self.euclidean_array[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-self.goal[0]),2) + math.pow((left_new[1]-self.goal[1]),2))
						self.totalcost[left_new[0],left_new[1]]= self.cost[left_new[0],left_new[1]]+self.euclidean_array[left_new[0],left_new[1]]
	
	
			right1_new=self.ActionRight2(currNode[0],currNode[1],currNode[2])
			status=self.IsObstacle(right1_new[0],right1_new[1])
			flag=self.IsValid(right1_new[0],right1_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[right1_new[0],right1_new[1]]==0:
					self.visited[right1_new[0],right1_new[1]]=1
					visited_node.append(right1_new)
					Q.append(right1_new)
					self.parent_list[right1_new[0]][right1_new[1]]=currNode
					self.euclidean_array[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-self.goal[0]),2) + math.pow((right1_new[1]-self.goal[1]),2))
					self.cost[right1_new[0],right1_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
					self.totalcost[right1_new[0],right1_new[1]]= self.cost[right1_new[0],right1_new[1]]+self.euclidean_array[right1_new[0],right1_new[1]]
				else:
					if self.cost[right1_new[0],right1_new[1]]>(self.cost[currNode[0],currNode[1]]+math.sqrt(2)):
						self.cost[right1_new[0],right1_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
						self.parent_list[right1_new[0]][right1_new[1]]=currNode
						self.euclidean_array[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-self.goal[0]),2) + math.pow((right1_new[1]-self.goal[1]),2))
						self.totalcost[right1_new[0],right1_new[1]]= self.cost[right1_new[0],right1_new[1]]+self.euclidean_array[right1_new[0],right1_new[1]]
	
	
			right2_new=self.ActionRight3(currNode[0],currNode[1],currNode[2])
			status=self.IsObstacle(right2_new[0],right2_new[1])
			flag=self.IsValid(right2_new[0],right2_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[right2_new[0],right2_new[1]]==0:
					self.visited[right2_new[0],right2_new[1]]=1
					visited_node.append(right2_new)
					Q.append(right2_new)
					self.parent_list[right2_new[0]][right2_new[1]]=currNode
					self.euclidean_array[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-self.goal[0]),2) + math.pow((right2_new[1]-self.goal[1]),2))
					self.cost[right2_new[0],right2_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
					self.totalcost[right2_new[0],right2_new[1]]= self.cost[right2_new[0],right2_new[1]]+self.euclidean_array[right2_new[0],right2_new[1]]
				else:
					if self.cost[right2_new[0],right2_new[1]]>(self.cost[currNode[0],currNode[1]]+math.sqrt(2)):
						self.cost[right2_new[0],right2_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
						self.parent_list[right2_new[0]][right2_new[1]]=currNode
						self.euclidean_array[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-self.goal[0]),2) + math.pow((right2_new[1]-self.goal[1]),2))
						self.totalcost[right2_new[0],right2_new[1]]= self.cost[right2_new[0],right2_new[1]]+self.euclidean_array[right2_new[0],right2_new[1]]
			
			
			left1_new=self.ActionLeft2(currNode[0],currNode[1],currNode[2])
			status=self.IsObstacle(left1_new[0],left1_new[1])
			flag=self.IsValid(left1_new[0],left1_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[left1_new[0],left1_new[1]]==0:
					self.visited[left1_new[0],left1_new[1]]=1
					visited_node.append(left1_new)
					Q.append(left1_new)
					self.parent_list[left1_new[0]][left1_new[1]]=currNode
					self.euclidean_array[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-self.goal[0]),2) + math.pow((left1_new[1]-self.goal[1]),2))
					self.cost[left1_new[0],left1_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
					self.totalcost[left1_new[0],left1_new[1]]= self.cost[left1_new[0],left1_new[1]]+self.euclidean_array[left1_new[0],left1_new[1]]
				else:
					if self.cost[left1_new[0],left1_new[1]]>(self.cost[currNode[0],currNode[1]]+math.sqrt(2)):
						self.cost[left1_new[0],left1_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
						self.euclidean_array[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-self.goal[0]),2) + math.pow((left1_new[1]-self.goal[1]),2))
						self.parent_list[left1_new[0]][left1_new[1]]=currNode
						self.totalcost[left1_new[0],left1_new[1]]= self.cost[left1_new[0],left1_new[1]]+self.euclidean_array[left1_new[0],left1_new[1]]
	
			
			left2_new=self.ActionLeft3(currNode[0],currNode[1],currNode[2])
			status=self.IsObstacle(left2_new[0],left2_new[1])
			flag=self.IsValid(left2_new[0],left2_new[1])
			if ( ((status) and (flag)) == 1):
				if self.visited[left2_new[0],left2_new[1]]==0:
					self.visited[left2_new[0],left2_new[1]]=1
					visited_node.append(left2_new)
					Q.append(left2_new)
					self.parent_list[left2_new[0]][left2_new[1]]=currNode
					self.euclidean_array[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-self.goal[0]),2) + math.pow((left2_new[1]-self.goal[1]),2))
					self.cost[left2_new[0],left2_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
					self.totalcost[left2_new[0],left2_new[1]]= self.cost[left2_new[0],left2_new[1]]+self.euclidean_array[left2_new[0],left2_new[1]]
				else:
					if self.cost[left2_new[0],left2_new[1]]>(self.cost[currNode[0],currNode[1]]+math.sqrt(2)):
						self.cost[left2_new[0],left2_new[1]]=(self.cost[currNode[0],currNode[1]]+math.sqrt(2))
						self.parent_list[left2_new[0]][left2_new[1]]=currNode
						self.euclidean_array[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-self.goal[0]),2) + math.pow((left2_new[1]-self.goal[1]),2))
						self.totalcost[left2_new[0],left2_new[1]]= self.cost[left2_new[0],left2_new[1]]+self.euclidean_array[left2_new[0],left2_new[1]]

		if(NoPath == 1):
			print("There exists a path")
			print("Back tracking . . .")
		if(NoPath == 0):
			print("NO VALID PATH")
			return (visited_node, [])
		start=[aa,bb,cc]		
		self.path_find(start)
		# print('path',self.path)
		print("path found")
		print(len(self.path))

		return (visited_node, self.path)


	def vrep(self, backtrack_states):
		del backtrack_states[len(backtrack_states)-1]
		# print(backtrack_states)
		while(len(backtrack_states) > 0):
			moves = backtrack_states.pop()
			# print(backtrack_states)
			route = moves[3]
		



			vrep.simxFinish(-1)
			clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

			if clientID!=(-1):
				print("connected to remort API server")
				# 
				err_code,left_wheel = vrep.simxGetObjectHandle(clientID,"wheel_left_joint", vrep.simx_opmode_oneshot_wait)
				err_code,right_wheel = vrep.simxGetObjectHandle(clientID,"wheel_right_joint", vrep.simx_opmode_oneshot_wait)
		
				if route == 'Straight':
					print('Straight')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, self.rpm1*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, self.rpm1*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)

				elif route == 'FastStraight':
					print('FastStraight')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, self.rpm2*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, self.rpm2*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)


				elif route == 'Left1':
					print('Left1')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, self.rpm1*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)


				elif route == 'Left2':
					print('Left2')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, self.rpm2*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)


				elif route == 'Left3':
					print('Left3')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, self.rpm1*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, self.rpm2*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)


				elif route == 'Right1':
					print('Right1')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, self.rpm1*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)


				elif route == 'Right2':
					print('Right2')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, self.rpm2*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)
				


				elif route == 'Right3':
					print('Right3')
					vrep.simxSetJointTargetVelocity(clientID, left_wheel, self.rpm2*(math.pi/30), vrep.simx_opmode_streaming)
					vrep.simxSetJointTargetVelocity(clientID, right_wheel, self.rpm1*(math.pi/30), vrep.simx_opmode_streaming)
					time.sleep(0.1)


			else:
				print("not connected to server")
				sys.exit("could not connect")

		vrep.simxSetJointTargetVelocity(clientID, left_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(clientID, right_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
		returnCode,handle = vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)
		returnCode,position = vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_blocking)
		print("completed")