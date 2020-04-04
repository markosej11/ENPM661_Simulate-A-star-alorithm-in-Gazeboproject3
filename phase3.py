# header files
import cv2
import numpy as np 
import math
from heapq import heappush, heappop, heapify



# class for Astar
class Astar(object):
    # init function
    def __init__(self, start, goal, startOrientation, clearance, rpm1, rpm2, radius, wheelRadius, length, dt):
        self.start = start
        self.goal = goal
        self.k = startOrientation
        self.numRows = 1020
        self.numCols = 1020
        self.i = 0
        self.j = 0
        self.clearance = clearance
        self.rpm1 = rpm1
        self.rpm2 = rpm2
        self.radius = radius
        self.wheelRadius = wheelRadius
        self.length = length
        self.dt = dt
       
        
    # move is valid 
    def IsValid(self, currRow, currCol):
        return (currRow <= 1010 - (self.radius + self.clearance)) and (currRow >= 10 + (self.radius + self.clearance)) and (currCol <= 1010 - (self.radius + self.clearance)) and (currCol >= 10 + (self.radius + self.clearance))             

    # checks for an obstacle
    def IsObstacle(self, row, col):
        safe_dist = self.clearance + self.radius
        circle1 = (row - 510)**2 + (col - 510)**2 - (100+safe_dist)**2 <= 0
        circle2 = (row - (210))**2 + (col - (710))**2 - (100+safe_dist)**2 <= 0
        circle3 = (row - (810))**2 + (col - (710))**2 - (100+safe_dist)**2 <= 0
        circle4 = (row - (810))**2 + (col - (310))**2 - (100+safe_dist)**2 <= 0
        square1 = row <= 285 + safe_dist and row >= 135 - safe_dist and col <= 385 + safe_dist and col >= 235 - safe_dist
        square2 = row <= 585 + safe_dist and row >= 435 -safe_dist and col <= 185 + safe_dist and col >= 35 -safe_dist
        square3 = row <= 585 + safe_dist and row >= 435 -safe_dist and col <= 985 + safe_dist and col >= 835 -safe_dist

        if (circle1 or circle2 or circle3 or circle4 or square1 or square2 or square3 ):
            return True
        return False

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
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    # action move FastStraight
    def ActionFastStraight(self, currRow, currCol, currangle):
        newtheta, dx, dy = self.Differential_motion(r2,r2,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    # action move Left1
    def ActionLeft1(self, currRow, currCol, currangle):
        newtheta, dx, dy = self.Differential_motion(0,r1,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False
    
    # action move Left2
    def ActionLeft2(self, currRow, currCol, currangle):
        newtheta, dx, dy = self.Differential_motion(0,r2,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    # action move Left3
    def ActionLeft3(self, currRow, currCol, currangle):
        newtheta, dx, dy = self.Differential_motion(r1,r2,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    # action move Right1
    def ActionRight1(self, currRow, currCol, currangle):
        newtheta, dx, dy = self.Differential_motion(r1,0,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    # action move right2
    def ActionRight2(self, currRow, currCol, currangle):
        newtheta, dx, dy = self.Differential_motion(r2,0,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    # action move Right3
    def ActionRight3(self, currRow, currCol, currangle):
        newtheta, dx, dy = self.Differential_motion(r2,r1,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        if(newtheta == 360):
            newtheta = 0
        self.k = newtheta
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    #Checking if goal node reached or not
    def CheckIfGoal(self, currRow, currCol, angle):
        check = (((currRow - self.goal[0]) * (currRow - self.goal[0])) + ((currCol - self.goal[1]) * (currCol - self.goal[1])) - ( 2 * 2))
        if(check <= 0):
            global cat
            global dog
            global bird
            cat = currRow
            dog = currCol
            bird = angle
            # print(cat,"cat")
            # print(dog,"dog")
            # print(bird,"bird")
            print("goal reached")
            return True
        else:
            return False


       
    # astar algorithm
    def Astar(self):
        # Converting RPM to radian per second
        global r1
        global r2
        r1 = (2*22*self.rpm1)/(60*7)
        r2 = (2*22*self.rpm2)/(60*7)
        print("Creating numpy arrays")
        distMap = np.full((int(self.numRows*1), int(self.numCols*1)), np.inf)
        # path = np.full((int(self.numRows*1), int(self.numCols*1),360), -1)
        path = [[ [-1 for col in range(360)] for col in range(1020)] for row in range(1020)]
        visited = np.full((int(self.numRows*1), int(self.numCols*1)), 0)
        print("Searching. May take upto 10 min")
        explored_states = []
        queue = []
        aa = self.start[0]
        bb = self.start[1]
        cc = self.k
        start1 = (aa,bb,self.k)
        heappush(queue, (0, start1))
        x1 = int(self.start[0]*1)
        y1 = int(self.start[1]*1)
        distMap[x1][y1] = 0

        while(len(queue) > 0):
            NoPath = 0
            heapify(queue)
            _, currNode = heappop(queue)
            x = int(currNode[0]*1)
            y = int(currNode[1]*1)
            visited[x][y] = 1
            explored_states.append(currNode)
            # if goal node then exit
            if(self.CheckIfGoal(currNode[0],currNode[1],currNode[2]) == True):
                NoPath = 1
                # print('goal')
                break
            h_dist = (((currNode[0] - self.goal[0]) ** 2) + 
                       ((currNode[1] - self.goal[1]) ** 2))
            h_dist = round(math.sqrt(h_dist),2)
            
            #Action sets
            if(self.ActionStraight(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode 
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionLeft1(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode 
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionLeft2(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionLeft3(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))
                    
            if(self.ActionFastStraight(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionRight1(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionRight2(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode             
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionRight3(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)][self.k] = currNode 
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))             
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))
                    

        f1 = self.goal[0]
        f2 = self.goal[1]
        if(NoPath == 1):
            print("There exists a path")
            print("Back tracking . . .")
        if(NoPath == 0):
            print("NO VALID PATH")
            return (explored_states, [], distMap[int(f1*1)][int(f2*1)])
        
        # backtrack path
        result = (cat, dog, bird)
        backtrack_states = []
        node = result
        while(path[node[0]][node[1]][node[2]] != -1):
            backtrack_states.append(node)
            node = path[node[0]][node[1]][node[2]]
        backtrack_states.append(self.start)
        backtrack_states = list(reversed(backtrack_states))
        print("Back tracking done") 
        return (explored_states, backtrack_states, distMap[cat][dog])

    # animate path
    def animate(self, explored_states, backtrack_states, path):
        print("Animation starting")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(path), fourcc, 20.0, ((self.numCols*1),(self.numRows*1)))
        image = np.zeros((int(self.numRows*1), int(self.numCols*1), 3), dtype=np.uint8)
        for row in range(0, self.numRows):
            for col in range(0, self.numCols):
                if(self.IsValid((col), (row)) and self.IsObstacle((col), (row)) == False):
                    image[(col),(row)] = (0, 128, 255)
    
        count = 0
        image = cv2.circle(image, (int(self.start[1]),int(self.start[0])), 5, (255,255,255), 5)
        image = cv2.circle(image, (int(self.goal[1]),int(self.goal[0])), 5, (255,0,165), 5)

        for state in explored_states:
            image[int((state[0]*1)),int((state[1]*1))] = (255, 255, 0)
            if(count%75 == 0):
                out.write(image)
            count = count + 1
        
        if(len(backtrack_states) > 0):
            for state in backtrack_states:
                image = cv2.circle(image, (int(state[1]),int(state[0])), 1, (0,0,255), 1)
                out.write(image)
        height = int(image.shape[0] * 0.5)
        width = int(image.shape[1] * 0.5)
        dim = (width,height)
        resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow('result', resized)
                
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()