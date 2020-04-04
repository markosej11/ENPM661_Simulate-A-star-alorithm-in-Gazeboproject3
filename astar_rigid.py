
# header files
from phase3 import *
import sys

startCol = float(input("Enter the x coordinate for start node (between -5.00 and 5.00) : "))
# startCol = -4
startCol = round(startCol,2)
print("Entered x coordinate is : ",startCol)
startCol = (startCol + 5.1)*100
startCol = round(startCol)
print(startCol)

startRow = float(input("Enter the y coordinate for start node (between -5.00 and 5.00) : "))
# startRow = -4
startRow = round(startRow,2)
print("Entered x coordinate is : ",startRow)
startRow = (10.2-(startRow + 5.1))*100
startRow = round(startRow)
print(startRow)

ori = float(input("Enter the Orientation of the robot : "))
ori = round(ori)
ori = int(ori)
if((ori<=0) or (ori>360)):
    flag1 = 0
    while(flag1 == 0):
        ori = int(input("Please enter orientation in between 0 and 359 : "))
        if((ori>=0) and (ori<360)):
            flag1 = 1
print("Entered orientation is : ",ori)
# startOrientation = 0

goalCol = float(input("Enter the x coordinate for goal node (between -5.00 and 5.00) : "))
# goalCol = 4
goalCol = round(goalCol,2)
print("Entered x coordinate is : ",goalCol)
goalCol = (goalCol + 5.1)*100
goalCol = round(goalCol)
print(goalCol)

goalRow = float(input("Enter the y coordinate for goal node (between -5 and 5) : "))
# goalRow = 4
goalRow = round(goalRow,2)
print("Entered x coordinate is : ",goalRow)
goalRow = (10.2-(goalRow + 5.1))*100
goalRow = round(goalRow)
print(goalRow)

# radius = int(input("Enter the radius for the robot : "))
clearance = 10
choice = input("The default clearance for the robot is 10 CM, do you want to increase the clearance? (y/n) ")
if((choice == 'y') or (choice == 'Y')):
    flag = 1
    while(flag == 1):
        c2 = float(input("Enter the new clearance : "))
        if(c2<=10):
            flag = 1
            print('clearance should be greater than 10 CM. Please reenter!')
        else:
            flag = 0
            clearance = c2
# clearance = float(input("Enter the clearance for the robot in meters: "))
rpm1 = float(input("Enter velocity 1 in RPM (max is 163) : "))
# maxium is 163 RPM
# rpm1 = 60
rpm2 = float(input("Enter velocity 2 in RPM (max is 163) : "))
# rpm2 = 30

radius = 17.7  #meters
wheelRadius = 3.8  #meters
length = 35.4  #need to change
dt = 0.5


# take start and goal node as input
start = (startRow, startCol)
goal = (goalRow, goalCol)
astar = Astar(start, goal, ori, clearance, rpm1, rpm2, radius, wheelRadius ,length, dt )

if(astar.IsValid(start[0], start[1])):
    print(start[0],start[1])
    if(astar.IsValid(goal[0], goal[1])):
        if(astar.IsObstacle(start[0],start[1]) == False):
            if(astar.IsObstacle(goal[0], goal[1]) == False):
                (explored_states, backtrack_states, distance_from_start_to_goal) = astar.Astar()
                astar.animate(explored_states, backtrack_states, "./astar_rigid.avi")
                if(distance_from_start_to_goal == float('inf')):
                    print("\nNo optimal path found.")
                else:
                    print("\nOptimal path found. Distance is " + str(distance_from_start_to_goal))
            else:
                print("The entered goal node is an obstacle ")
                print("Please check README.md file for running Astar_rigid.py file.")
        else:
            print("The entered initial node is an obstacle ")
            print("Please check README.md file for running Astar_rigid.py file.")
    else:
        print("The entered goal node outside the map ")
        print("Please check README.md file for running Astar_rigid.py file.")
else:
    print("The entered initial node is outside the map ")
    print("Please check README.md file for running Astar_rigid.py file.")
