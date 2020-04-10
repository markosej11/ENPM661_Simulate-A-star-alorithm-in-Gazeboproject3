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
1. Two py files are required to run the program. (astar_rigid.py and phase3.py).
2. make sure both the file are in the same directory.
3. We recommend you to run the program using the terminal in linux.
4. Run the file named astar_rigid.py to execute the code. Use below comment to run code - 
	python3 astar_rigid.py
		
5. The program will first ask you to enter the start x , y and orientation of the robot.
6. The program wil then ask you to enter the goal x and y.
7. Next the program will ask if you want to increase the clerance from 10cm. Enter 'y' or 'Y' to increase the clearance.
8. The program will then ask for the RPM.
9. once all these are entered you have to wait for about 1 min and you will get the output



Py files needed:
Phase3.py
astar_rigid.py  (run this file)



Result:
The blue region is the explored region. The green orange is the unexplored region. And lastly, the red region is the path from the start node to goal node.
If there is no solution then the output will be 'No valid path'
if there is a solution you can see the path in the pop up


For example:
For the start node (-4, -4, 60), goal node (4, 4), clearance = 1, RMP1 = 60, RPM2 = 40, the time taken is 5 seconds.


Software Required:
To run the .py files, use Python 3. 
Standard Python 3 libraries like numpy, math and OpenCV are used.
