import vrep
import time
import sys
import math

rpm1 = 60
rpm2 = 40
print(rpm1*(math.pi/30))
print(rpm2*(math.pi/30))
answer = ['Right3','Right1','Straight','Right3','Right1','Straight','Right3','Right1','Straight','Right3','Right1','Straight','Right3','Right1','Straight','Left3','Right3','Right3','Right3','Straight','Left1','Right1']
print(answer)
while(len(answer) >0):
	moves = answer.pop()
	# print('answer',answer)
	route = moves
		



	vrep.simxFinish(-1)
	clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

	if clientID!=(-1):
		print("connected to remort API server")
				# 
		err_code,left_wheel = vrep.simxGetObjectHandle(clientID,"wheel_left_joint", vrep.simx_opmode_oneshot_wait)
		err_code,right_wheel = vrep.simxGetObjectHandle(clientID,"wheel_right_joint", vrep.simx_opmode_oneshot_wait)
		
		if route == 'Straight':
			print('Straight')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, rpm1*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, rpm1*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)

		elif route == 'FastStraight':
			print('FastStraight')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, rpm2*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, rpm2*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)


		elif route == 'Left1':
			print('Left1')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, rpm1*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)


		elif route == 'Left2':
			print('Left2')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, rpm2*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)


		elif route == 'Left3':
			print('Left3')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, rpm1*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, rpm2*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)


		elif route == 'Right1':
			print('Right1')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, rpm1*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)


		elif route == 'Right2':
			print('Right2')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, rpm2*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)
				


		elif route == 'Right3':
			print('Right3')
			vrep.simxSetJointTargetVelocity(clientID, left_wheel, rpm2*(math.pi/30), vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, right_wheel, rpm1*(math.pi/30), vrep.simx_opmode_streaming)
			time.sleep(0.5)


	else:
		print("not connected to server")
		sys.exit("could not connect")

vrep.simxSetJointTargetVelocity(clientID, left_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, right_wheel, 0*(math.pi/30), vrep.simx_opmode_streaming)
returnCode,handle = vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)
returnCode,position = vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_blocking)
print("completed")
