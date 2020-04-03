
# header files
from ashwin import *
import sys

# startCol = float(input("Enter the x coordinate for start node (between -5 and 5) : "))
startCol = -4
startCol = (startCol + 5.1)*100
startCol = round(startCol)
print(startCol)

# startRow = float(input("Enter the y coordinate for start node (between -5 and 5) : "))
startRow = -4
startRow = (10.2-(startRow + 5.1))*100
startRow = round(startRow)
print(startRow)

# ori= int(input("Enter the Orientation of the robot (0/30/60/90/120/150/180/210/240/270/300/330) : "))
# ori = 0
# startOrientation = ((22 * ori) / (7 * 180))
startOrientation = 0
# goalCol = float(input("Enter the x coordinate for goal node (between -5 and 5) : "))
goalCol = 4
goalCol = (goalCol + 5.1)*100
goalCol = round(goalCol)
print(goalCol)
# goalRow = float(input("Enter the y coordinate for goal node (between -5 and 5) : "))
goalRow = 4
goalRow = (10.2-(goalRow + 5.1))*100
goalRow = round(goalRow)
print(goalRow)

# radius = int(input("Enter the radius for the robot : "))
clearance = 2.3
choice = input("The default clearance for the robot is 2.3 CM, do you want to increase the clearance? (y/n) ")
if((choice == 'y') or (choice == 'Y')):
    flag = 1
    while(flag == 1):
        c2 = float(input("Enter the new clearance "))
        if(c2<=2.3):
            flag = 1
            print('clearance should be greater than 0.1 meters. Please reenter!')
        else:
            flag = 0
            clearance = c2
# clearance = float(input("Enter the clearance for the robot in meters: "))
# rpm1 = float(input("Enter velocity 1 in RPM : "))
# maxium is 163 RPM
rpm1 = 60
# rpm2 = float(input("Enter velocity 2 in RPM : "))
rpm2 = 30

radius = 17.7  #meters
wheelRadius = 3.8  #meters
length = 35.4  #need to change
dt = 0.5


# take start and goal node as input
start = (startRow, startCol)
goal = (goalRow, goalCol)
astar = Astar(start, goal, startOrientation, clearance, rpm1, rpm2, radius, wheelRadius ,length, dt )

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
