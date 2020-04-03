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
        # constants
        safe_dist = self.clearance + self.radius
        

        # circle1 = (row - 5.1)**2 + (col - 5.1)**2 - (1.0000+safe_dist)**2 <= 0
        # circle2 = (row - (3.1))**2 + (col - (2.1))**2 - (1.0000+safe_dist)**2 <= 0
        # circle3 = (row - (7.1))**2 + (col - (2.1))**2 - (1.0000+safe_dist)**2 <= 0
        # circle4 = (row - (7.1))**2 + (col - (8.1))**2 - (1.0000+safe_dist)**2 <= 0
        # square1 = row <= 1.6 + safe_dist and row >= 0.35 - safe_dist and col <= 5.85 + safe_dist and col >= 4.35 - safe_dist
        # square2 = row <= 9.85 + safe_dist and row >= 8.35 -safe_dist and col <= 5.85 + safe_dist and col >= 4.35 -safe_dist
        # square3 = row <= 3.85 + safe_dist and row >= 2.35 -safe_dist and col <= 8.85 + safe_dist and col >= 7.35 -safe_dist

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


    def Differential_motion(self,r1,r2,theta):
        thata = theta * ( 22 / ( 7 * 180 ) )
        dtheta=(self.wheelRadius*(r2-r1)*self.dt)/self.length + theta
        dtheta= dtheta * ( (7 * 180) / 22) 
        dtheta=dtheta%360
        # print(dtheta)
        # print(r1,r2)
        dx=(self.wheelRadius*(r1+r2)*math.cos(dtheta * (22 / (7 * 180)))*self.dt)/2
        dy=(self.wheelRadius*(r1+r2)*math.sin(dtheta * (22 / (7 * 180)))*self.dt)/2
        
        # print("dx,dy")
        # print(dx)
        # print(dy)
        return dtheta,dx,dy


    # action move left
    def ActionStraight(self, currRow, currCol, currangle):
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        # print("Moved Straight")
        newtheta, dx, dy = self.Differential_motion(r1,r1,currangle)
        # print(dx,"dx",dy,"dy",newtheta,"theta")

        dx = round(dx)
        dy = round(dy)
        # print(dx,"dx",dy,"dy",newtheta,"theta")
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i,"dx")
        # print(self.j,"dy")
        # print(self.k,"newtheta")

        # print('new')
        # print(currRow,currCol,newtheta)

        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False


    def ActionFastStraight(self, currRow, currCol, currangle):
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        # print('Moved FastStraight')
        newtheta, dx, dy = self.Differential_motion(r2,r2,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i)
        # print(self.j)
        # print(self.k)
    

        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False


    def ActionLeft1(self, currRow, currCol, currangle):
        # print("Moved left1")
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        newtheta, dx, dy = self.Differential_motion(0,r1,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i,"dx")
        # print(self.j,"dy")
        # print(self.k,"newtheta")
        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    def ActionLeft2(self, currRow, currCol, currangle):
        # print("Moved left2")
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        newtheta, dx, dy = self.Differential_motion(0,r2,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i)
        # print(self.j)
        # print(self.k)

        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False


    def ActionLeft3(self, currRow, currCol, currangle):
        # print("Moved left3")
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        newtheta, dx, dy = self.Differential_motion(r1,r2,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i)
        # print(self.j)
        # print(self.k)

        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    def ActionRight1(self, currRow, currCol, currangle):
        # print("Moved Right1")
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        newtheta, dx, dy = self.Differential_motion(r1,0,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i)
        # print(self.j)
        # print(self.k)

        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    def ActionRight2(self, currRow, currCol, currangle):
        # print("Moved right2")
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        newtheta, dx, dy = self.Differential_motion(r2,0,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i)
        # print(self.j)
        # print(self.k)

        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False

    def ActionRight3(self, currRow, currCol, currangle):
        # print("Moved Right3")
        # print(currRow,currCol,currangle,"currRow ,  currCol, currangle")
        newtheta, dx, dy = self.Differential_motion(r2,r1,currangle)
        dx = round(dx)
        dy = round(dy)
        newtheta = round(newtheta)
        currRow = currRow + dx
        currCol = currCol + dy
        self.i = dx
        self.j = dy
        self.k = newtheta
        # print(self.i)
        # print(self.j)
        # print(self.k)

        if(self.IsValid(currRow , currCol) and self.IsObstacle(currRow , currCol) == False):
            return True
        return False















    


    def CheckIfGoal(self, currRow, currCol):
        check = (((currRow - self.goal[0]) * (currRow - self.goal[0])) + ((currCol - self.goal[1]) * (currCol - self.goal[1])) - ( 2 * 2))
        # print(check,"check")
        if(check <= 0):
            global cat
            global dog
            cat = currRow
            dog = currCol
            print(cat)
            print(dog)
            print("goal reached")
            return True
        else:
            return False


    
    # astar algorithm
    def Astar(self):
        # create hashmap to store distances
        global r1
        global r2
        r1 = (2*22*self.rpm1)/(60*7)
        r2 = (2*22*self.rpm2)/(60*7)

        # image = np.zeros((int(self.numRows), int(self.numCols), 3), dtype=np.uint8)
        # for row in range(0, self.numRows):
        #     for col in range(0, self.numCols):
        #         if(self.IsValid((row), (col)) and self.IsObstacle((row), (col)) == False):
        #             image[row,col] = (154, 250, 0)

        # # image = cv2.circle(image, (int(self.start[0]),int(self.start[1])), 20, (255,255,255), 5)
        # image = cv2.circle(image, (int(self.goal[0]),int(self.goal[1])), 20, (128,255,128), 5)

        # height = int(image.shape[0] * 1)
        # width = int(image.shape[1] * 1)
        # dim = (width,height)
        # resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        # cv2.imshow('result', resized)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # out.release()





      


        distMap = np.full((int(self.numRows*1), int(self.numCols*1)), np.inf)
        path = np.full((int(self.numRows*1), int(self.numCols*1)), -1)
        visited = np.full((int(self.numRows*1), int(self.numCols*1)), 0)            
        # create queue, push the source and mark distance from source to source as zero
        explored_states = []
        queue = []
        aa = self.start[0]
        bb = self.start[1]
        start1 = (aa,bb,self.k)
        heappush(queue, (0, start1))
        # print(queue,'first queue')
        x1 = int(self.start[0]*1)
        y1 = int(self.start[1]*1)
        distMap[x1][y1] = 0
        check = -1

        while(len(queue) > 0):
            check = check + 1
            NoPath = 0
            # print(check)
            # if(check>=50000000):
            #     print(explored_states,"explored_states")
            #     print("BREAKING")
            #     break
            heapify(queue)
            # print(queue)
            _, currNode = heappop(queue)
            # print('currnode 1')
            # print(currNode[0])
            # print(currNode[1])
            # print(currNode[2])
            x = int(currNode[0]*1)
            y = int(currNode[1]*1)
            # print(x,y)
            # else:
            #     x = int(currNode[0])
            #     y = int(currNode[1])
            # print('x and y')
            # print(x,y)
            # print('')
            # print('')
            visited[x][y] = 1
            explored_states.append(currNode)
            # print(explored_states,'explored states')
        
            # if goal node then exit
            if(self.CheckIfGoal(currNode[0],currNode[1]) == True):
                NoPath = 1
                print('goal')
                # print(explored_states,'explored states')
                # print(queue,'first queue')
                break


            h_dist = (((currNode[0] - self.goal[0]) ** 2) + 
                       ((currNode[1] - self.goal[1]) ** 2))
            h_dist = round(math.sqrt(h_dist),2)
            


            if(self.ActionStraight(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
        
                # print(cost,"cost")
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    # print(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)])
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5 
                    # print(path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)]) #Five is wrong              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))
                    # print(queue)
                    # break


            if(self.ActionLeft1(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5  #Five is wrong 
                    # print(self.i,self.j)             
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))
                    # print(queue)
                    # break

            if(self.ActionLeft2(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5  #Five is wrong              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionLeft3(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5  #Five is wrong              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))
                    

            if(self.ActionFastStraight(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5  #Five is wrong              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionRight1(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5  #Five is wrong              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionRight2(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5  #Five is wrong              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))

            if(self.ActionRight3(currNode[0], currNode[1], currNode[2]) and visited[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] == 0 ): 
                cost = round(math.sqrt(((currNode[0] - (currNode[0] + self.i)) ** 2) + ((currNode[1] - (currNode[1] + self.j))**2)),2)
                if(distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] > (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + h_dist+cost)):
                    distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = (distMap[int(currNode[0]*1)][int(currNode[1]*1)] + round(h_dist+cost,2))
                    path[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)] = 5  #Five is wrong              
                    heappush(queue, (distMap[int((currNode[0] + self.i)*1)][int((currNode[1] + self.j)*1)], (round(currNode[0] + self.i,3), round(currNode[1] + self.j,3),self.k) ))
                    # print(queue)
                    # break



           
            
            
            



        # return if no optimal path
        # checker = []
        f1 = self.goal[0]
        f2 = self.goal[1]
        # for a in np.arange(f1-0.25,f1+0.25,0.001):
        #     for b in np.arange(f2-0.25,f2+0.25,0.001):
        #         checker.append(distMap[int(a*1)][int(b*1)])
        # NoPath = 0
        # ans = float('inf')
        # for a in range(len(checker)):
        #     if(checker[a] != ans):
        #         print("There exists a path")
        #         NoPath = 1
        #         break
        if(NoPath == 1):
            print("There exists a path")
        if(NoPath == 0):
            print("NO VALID PATH")
            return (explored_states, [], distMap[int(f1*1)][int(f2*1)])





    #     # for a in np.arange(f1-1,f1+1.5,0.5):
    #     #     for b in np.arange(f2-1,f2+1.5,0.5):
    #     #         for c in range(0,360,30):
    #     #             check.append(distMap[a,b])
    #     # for c in range(0,360,30):
    #     #     check.append(distMap[f1,f2-1.5,c])
    #     # for c in range(0,360,30):
    #     #     check.append(distMap[f1,f2+1.5,c])
    #     # for c in range(0,360,30):
    #     #     check.append(distMap[f1+1.5,f2,c])
    #     # for c in range(0,360,30):
    #     #     check.append(distMap[f1-1.5,f2,c])
    #     # NoPath = 0
    #     # ans = float('inf')
    #     # for a in range(len(check)):
    #     #     if(check[a] != ans):
    #     #         print("There exists a path")
    #     #         NoPath = 1
    #     #         break
    #     # if(NoPath == 0):
    #     #     print("NO VALID PATH")
    #     #     return (explored_states, [], distMap[f1,f2,0])

        # for a in range(0,360,30):
        #     if(distMap[cat,dog,a] != ans):
        #         bird = a

        # print(distMap[cat,dog,bird],"answer")
        # result = (cat, dog, bird)
        
        # # backtrack path
        # backtrack_states = []
        # node = result
        # while(path[node] != -1):
        #     backtrack_states.append(node)
        #     node = path[node]
        # backtrack_states.append(self.start)
        # backtrack_states = list(reversed(backtrack_states))
        # # print(explored_states) 
        # return (explored_states, backtrack_states, distMap[cat,dog,bird])
        return (explored_states, [], distMap[int(f1*1)][int(f2*1)])
    




    # animate path
    def animate(self, explored_states, backtrack_states, path):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(path), fourcc, 20.0, ((self.numCols*1),(self.numRows*1)))
        image = np.zeros((int(self.numRows*1), int(self.numCols*1), 3), dtype=np.uint8)
        count = 0
        for row in range(0, self.numRows):
            for col in range(0, self.numCols):
                if(self.IsValid((col), (row)) and self.IsObstacle((col), (row)) == False):
                    image[(col),(row)] = (0, 128, 255)
                    # if(count%75 == 0):
                    # out.write(image)
                    count = count + 1
        count = 0
        image = cv2.circle(image, (int(self.start[1]),int(self.start[0])), 5, (255,255,255), 5)
        image = cv2.circle(image, (int(self.goal[1]),int(self.goal[0])), 5, (255,0,165), 5)

        for state in explored_states:
            # print(state)
            image[int((state[0]*1)),int((state[1]*1))] = (255, 255, 0)
            if(count%75 == 0):
                out.write(image)
            count = count + 1
        










        # if(len(backtrack_states) > 0):
        #     for state in backtrack_states:
        #         image[int(self.numRows - state[0]), int(state[1] - 1)] = (0, 0, 255)
        #         # out.write(image)
        #         cv2.imshow('result', image)
        #         cv2.waitKey(5)
        height = int(image.shape[0] * 1)
        width = int(image.shape[1] * 1)
        dim = (width,height)
        resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow('result', resized)
                
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()

        # image = np.zeros((int(self.numRows), int(self.numCols), 3), dtype=np.uint8)
        # for row in range(0, self.numRows):
        #     for col in range(0, self.numCols):
        #         if(self.IsValid((row), (col)) and self.IsObstacle((row), (col)) == False):
        #             image[row,col] = (154, 250, 0)

        # image = cv2.circle(image, (int(self.start[0]),int(self.start[1])), 5, (255,255,255), 5)
        # image = cv2.circle(image, (int(self.goal[0]),int(self.goal[1])), 5, (128,255,128), 5)

        # height = int(image.shape[0] * 1)
        # width = int(image.shape[1] * 1)
        # dim = (width,height)
        # resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        # cv2.imshow('result', resized)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # out.release()



