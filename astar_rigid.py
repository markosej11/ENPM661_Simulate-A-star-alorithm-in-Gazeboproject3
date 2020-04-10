
# header files
from phase3 import *


#Taking inputs from user
startRow = float(input("Enter the x coordinate for start node (between -5.00 and 5.00) : "))
startRow = round(startRow,2)
print("Entered x coordinate is : ",startRow)
startRow = (startRow + 5.1)*100
startRow = round(startRow)

startCol = float(input("Enter the y coordinate for start node (between -5.00 and 5.00) : "))
startCol = round(startCol,2)
print("Entered x coordinate is : ",startCol)
startCol = (10.2-(startCol + 5.1))*100
startCol = round(startCol)

#Taking orientation from user
ori = float(input("Enter the Orientation of the robot : "))
ori = round(ori)
ori = int(ori)
if((ori<0) or (ori>=360)):
    flag1 = 0
    while(flag1 == 0):
        ori = int(input("Please enter orientation in between 0 and 359 : "))
        if((ori>=0) and (ori<360)):
            flag1 = 1
print("Entered orientation is : ",ori)

#Taking goal nodes from user
goalRow = float(input("Enter the x coordinate for goal node (between -5.00 and 5.00) : "))
goalRow = round(goalRow,2)
print("Entered x coordinate is : ",goalRow)
goalRow = (goalRow + 5.1)*100
goalRow = round(goalRow)

goalCol = float(input("Enter the y coordinate for goal node (between -5 and 5) : "))
goalCol = round(goalCol,2)
print("Entered x coordinate is : ",goalCol)
goalCol = (10.2-(goalCol + 5.1))*100
goalCol = round(goalCol)

#Taking clerance from user, Default clerance is 10 cm
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

#Taking RPM from user
rpm1 = float(input("Enter velocity 1 in RPM (max is 163) : "))
rpm2 = float(input("Enter velocity 2 in RPM (max is 163) : "))


radius = 17.7  #Radius of bot in cm
wheelRadius = 3.8 #Wheel radius in meters 
length = 23 #Length of bot
dt = 0.5 #Time intravel



start = (startRow, startCol)
goal = (goalRow, goalCol)
astar = Astar(start, goal, ori, clearance, rpm1, rpm2, radius, wheelRadius ,length, dt )
ans1 = astar.IsValid(start[0], start[1])
if(ans1==1):

    ans1 = astar.IsValid(goal[0], goal[1])
    if(ans1==1):

        ans2 = astar.IsObstacle(start[0],start[1])
        if(ans2==1):

            ans2 = astar.IsObstacle(goal[0],goal[1])
            if(ans2==1):
                (explored_states, backtrack_states) = astar.Astar()
                astar.animate(explored_states, backtrack_states, "./astar_rigid.avi")
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
