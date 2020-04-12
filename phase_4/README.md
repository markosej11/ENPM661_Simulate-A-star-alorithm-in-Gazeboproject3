# ENPM_661_Phase_3_-_4_project3
Project 3 phase - ENPM661:


Authors:
Nupur Nimbekar, Markose Jacob (Group 16)

User inputs:
1. X coordinate for start
2. Y coordinate for start
3. Starting orientation
4. X coordinate for goal
5. Y coordidate for goal
6. Clearance
7. RPM1
8. RPM2


Instructions for running the code:
1. First open vrep and open the map (map.ttt) which can be found in Vrep folder.
2. We have used port 19999 for establishing the connection.
3. So make sure to change  "portIndex1_port             = 19999" in the remoteApiConnections.txt file which can be found in the vrep file on your computer.
3. We recommend you to run the program using the terminal in linux.
4. open the file named vrep in .../proj3_group16_VREP_python/phase_4/Vrep.
4. Run the file named astar_rigid.py to execute the code. Use below comment to run code - 
	python3 astar_rigid.py
		
5. The program will first ask you to enter the start x , y and orientation of the robot.
6. The program wil then ask you to enter the goal x and y.
7. Next the program will ask if you want to increase the clerance from 10cm. Enter 'y' or 'Y' to increase the clearance.
8. The program will then ask for the RPM.
9. once all these are entered you have to wait for about 1 min and you will be able to see the bot move



Py files needed:
astar_rigid.py  (run this file)
Phase3.py
complexCommandTest.py
depth_image_encoding.py
pathPlanningTest.py
ply.py
simpleSynchronousTest.py
simpleTest.py
visualization.py
vrep.py
vrepConst.py



Result:
If there is no solution then the output will be 'No valid path'
if there is a solution you can see the robot moving as per the solution found.



Software Required:
To run the .py files, use Python 3. 
Standard Python 3 libraries like numpy, math, time, sys and vrep are used.
