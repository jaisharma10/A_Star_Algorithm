## ------------------------------------------------------------------------------------------
#                                  A* Search [Maze]
## ------------------------------------------------------------------------------------------

'''
Author: Jai Sharma
Task: implement A* algorithm on a 16 x 8 maze between a given start and goal node

'''

## ------------------------------------------------------------------------------------------
#                                        Import Libraries
## ------------------------------------------------------------------------------------------

import time
import copy
import pygame
import sys
from scipy.spatial import distance
import math

start_time = time.time()
print("=======================================================================")

## ------------------------------------------------------------------------------------------
#                                     Node Class
## ------------------------------------------------------------------------------------------

class Node:
    
    '''
    Attributes:
        state: state of the node
        parent: parent of the node
        c2c: cost to come
        c2g: cost to go
        cost: total cost
    '''
    
    def __init__(self, state, parent, c2c, c2g, cost):
        self.state = state     # current node in the tree
        self.parent = parent   # parent of current node
        self.c2c = c2c         # cost to come
        self.c2g = c2g         # cost to go
        self.cost = cost       # total cost
        
    def __repr__(self):         # special method used to represent a class’s objects as string
        return(f' state: {self.state}, c2c: {self.c2c}, c2g: {self.c2g}, cost: {self.cost}')
    
    def moveUp(self, pos, goal): # Move to the node above
        row, col = pos[0], pos[1]
        if col < 8:  # node above exists
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row, col + 1
            newNode.c2c = round(self.c2c + 1, 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       # Up not possible

    def moveRight(self, pos, goal): 
        row, col = pos[0], pos[1]
        if row < 16:  
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row + 1, col 
            newNode.c2c = round(self.c2c + 1, 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)  

    def moveDown(self, pos, goal): 
        row, col = pos[0], pos[1]
        if col > 1:  
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row, col - 1
            newNode.c2c = round(self.c2c + 1, 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       

    def moveLeft(self, pos, goal): 
        row, col = pos[0], pos[1]
        if row > 1:  
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row - 1, col 
            newNode.c2c = round(self.c2c + 1, 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)  
        
    def moveUpRight(self, pos, goal): 
        row, col = pos[0], pos[1]
        if col < 8 and row < 16:  
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row + 1, col + 1
            newNode.c2c = round(self.c2c + math.sqrt(2), 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       # Up not possible
   
    def moveDownRight(self, pos, goal): 
        row, col = pos[0], pos[1]
        if col > 1 and row < 16:  # node above exists
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row + 1, col - 1
            newNode.c2c = round(self.c2c + math.sqrt(2), 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       
    
    def moveUpLeft(self, pos, goal): 
        row, col = pos[0], pos[1]
        if row > 1 and col < 8: 
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row - 1, col + 1
            newNode.c2c = round(self.c2c + math.sqrt(2), 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       
    
    def moveDownLeft(self, pos, goal): 
        row, col = pos[0], pos[1]
        if row > 1 and col > 1: 
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row - 1, col - 1
            newNode.c2c = round(self.c2c + math.sqrt(2), 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       
   
    def getNeighbours(self, pos, goal): # check for neighbours in the 8 directions
        neighbours = []
        
        up = self.moveUp(pos,goal) 
        down = self.moveDown(pos,goal) 
        left = self.moveLeft(pos,goal) 
        right = self. moveRight(pos,goal) 
        upRight = self. moveUpRight(pos,goal) 
        downRight = self. moveDownRight(pos,goal) 
        upLeft = self. moveUpLeft(pos,goal) 
        downLeft = self. moveDownLeft(pos,goal) 
        
        neighbours.append(up) if up else None
        neighbours.append(upRight) if upRight else None
        neighbours.append(right) if right else None
        neighbours.append(downRight) if downRight else None
        neighbours.append(down) if down else None
        neighbours.append(downLeft) if downLeft else None
        neighbours.append(left) if left else None
        neighbours.append(upLeft) if upLeft else None
        
        return(neighbours)
    
    def manhatten(self, pos, goal):
        dist = round(distance.euclidean(pos, goal),3)
        return(dist)
        
## ------------------------------------------------------------------------------------------
#                                         A* Function
## ------------------------------------------------------------------------------------------

def aStar(s, g, obsCord):
    
    pygame.init()
    magf = 50 # magnification factor
    screen = pygame.display.set_mode(((17)*magf, (9)*magf))
    hght = 9
    screen.fill((30,30,30))

    initialDistance = round(distance.euclidean(s,g),3)
    startNode = Node(s, None, 0, initialDistance, initialDistance)
    goalNode = Node(g, None, float('inf'), 0, float('inf'))

    queue = []                    # all neighbour states to explore
    visited = []                  # all visited lists fall here
    queue.append(startNode)       # add start node to queue
    visited.append(startNode.state)
    
    while queue != []:
        time.sleep(0.1)
        queue.sort(key = lambda x: x.cost)                # sort queue based on c2c
        currentNode = queue.pop(0)                        # pop node with lowest cost 

        # Visualize Maze Boundary
        boundary_colour = (0,0, 0)
        boundary_thickness = 35
        pygame.draw.line(screen, boundary_colour, (magf*(0), magf*(hght-0)), (magf*(0), magf*(hght-9)),boundary_thickness)
        pygame.draw.line(screen, boundary_colour, (magf*(0), magf*(hght-9)), (magf*(17), magf*(hght-9)),boundary_thickness)
        pygame.draw.line(screen, boundary_colour, (magf*(17), magf*(hght-9)), (magf*(17), magf*(hght-0)),boundary_thickness)
        pygame.draw.line(screen, boundary_colour, (magf*(17), magf*(hght-0)), (magf*(0), magf*(hght-0)),boundary_thickness)
        
        # Visualize obstacles in Maze
        wall_colour = (80,80,80)
        wall_thickness = 8
        pygame.draw.line(screen, wall_colour, (magf*(2), magf*(hght-1)), (magf*(2), magf*(hght-3)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(2), magf*(hght-5)), (magf*(2), magf*(hght-7)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(5), magf*(hght-5)), (magf*(5), magf*(hght-8)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(2), magf*(hght-3)), (magf*(5), magf*(hght-3)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(2), magf*(hght-5)), (magf*(5), magf*(hght-5)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(2), magf*(hght-7)), (magf*(3), magf*(hght-7)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(7), magf*(hght-2)), (magf*(7), magf*(hght-7)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(9), magf*(hght-4)), (magf*(9), magf*(hght-7)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(11), magf*(hght-4)), (magf*(11), magf*(hght-5)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(12), magf*(hght-1)), (magf*(12), magf*(hght-2)),wall_thickness)
        pygame.draw.polygon(screen, wall_colour, ((magf*(13), magf*(hght-2)),(magf*(13), magf*(hght-3)),(magf*(14), magf*(hght-3)),(magf*(14), magf*(hght-2))))
        pygame.draw.line(screen, wall_colour, (magf*(13), magf*(hght-5)), (magf*(13), magf*(hght-7)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(16), magf*(hght-2)), (magf*(16), magf*(hght-3)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(7), magf*(hght-7)), (magf*(15), magf*(hght-7)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(9), magf*(hght-4)), (magf*(11), magf*(hght-4)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(9), magf*(hght-2)), (magf*(14), magf*(hght-2)),wall_thickness)
        pygame.draw.line(screen, wall_colour, (magf*(13), magf*(hght-5)), (magf*(16), magf*(hght-5)),wall_thickness)

        pygame.draw.circle(screen, (0,128,0), (magf*(goalNode.state[0]), 9*magf-magf*goalNode.state[1]), 16)   # Goal Node
        pygame.draw.circle(screen, (255,0,0), (magf*(startNode.state[0]), 9*magf-magf*startNode.state[1]), 16) # Start Node
        pygame.draw.circle(screen, (255,255,255), (magf*(currentNode.state[0]), magf*(9-currentNode.state[1])), 7)   # Current Node

        pygame.display.update()
        
        # Case 1 --> Goal Reached
        if currentNode.state == goalNode.state:  # check if goal state reached
            print("Goal Reached !") 
            backTrackList = backtrack(currentNode, startNode)  # backtrack list is goal to start
            reversed_backTrackList = backTrackList[::-1] # reversed --> list is start to goal
            prev = reversed_backTrackList[0]
            print("backTrackList", backTrackList)
            for route in reversed_backTrackList:   # visualize the search algorithm
                pygame.draw.circle(screen, (0,0,250), (magf*(route[0]), 9*magf-magf*route[1]), 7)   # Current Node     
                pygame.draw.line(screen, (255, 255, 0), (magf*(route[0]), 9*magf-magf*route[1]), (magf*(prev[0]), 9*magf-magf*prev[1]),5)
                pygame.draw.circle(screen, (0,0,250), (magf*(prev[0]), 9*magf-magf*prev[1]), 7)   # Current Node     
                pygame.display.update()
                prev = route
            time.sleep(15)
            print("Cost to reach Goal Node -->", round(currentNode.cost, 3))
            break

        # Case 2: goal not reached, evaluate neighbours to popped current node 
        else: 
            Neighbours = currentNode.getNeighbours(currentNode.state, goalNode.state)  # get neighbours of current node
            for child in Neighbours:
                if child.state not in obsCord:
                    # Case2A: previosly explored, update if needed
                    if child.state in visited:
                        if child.parent != currentNode.state:
                            parentNode = child.parent     
                            if child.c2c > parentNode.c2c + 1: # update node if needed
                                child.c2c = parentNode.c2c + 1
                        
                    # Case2B: add to queue, previosly not explored
                    else:
                        queue.append(child)
                        visited.append(child.state)   
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
    return(None)
        
        
## ------------------------------------------------------------------------------------------
#                                  Helper Functions
## ------------------------------------------------------------------------------------------

def backtrack(current, start):
    backtrackList = [current.state]   # new list to collect backtracked list
    while(current.state != start.state):
        current = current.parent
        backtrackList.append(current.state)
    return(backtrackList)

def buildMap(mapHeight, mapWidth):
    mapCord = []
    obsCord = []
    
    for x in range(1, mapWidth + 1, 1):
        for y in range(1, mapHeight + 1,1):
            mapCord.append([x,y])

    for x,y in mapCord: 
        # Vertical Walls
        if (x == 2) and (y <= 7) and (y >= 5):  # Wall 1
            obsCord.append([x,y])    
        if (x == 2) and (y <= 3) and (y >= 1):  # Wall 2
            obsCord.append([x,y]) 
        if (x == 5) and (y <= 8) and (y >= 5):  # Wall 3
            obsCord.append([x,y]) 
        if (x == 7) and (y <= 7) and (y >= 2):  # Wall 4
            obsCord.append([x,y]) 
        if (x == 9) and (y <= 7) and (y >= 4):  # Wall 5
            obsCord.append([x,y])             
        if (x == 11) and (y <= 5) and (y >= 4):  # Wall 6
            obsCord.append([x,y])
        if (x == 12) and (y <= 2) and (y >= 1):  # Wall 7
            obsCord.append([x,y])              
        if (x == 13) and (y <= 7) and (y >= 5):  # Wall 8
            obsCord.append([x,y])   
        if (x == 16) and (y <= 3) and (y >= 2):  # Wall 9
            obsCord.append([x,y])               
        if (x == 13) and (y <= 3) and (y >= 2):  # Wall 10
            obsCord.append([x,y])   
        if (x == 14) and (y <= 3) and (y >= 2):  # Wall 11
            obsCord.append([x,y]) 
        # Horizontal Walls        
        if (x <= 5) and (x >= 2) and (y == 3):  # Wall 1
            obsCord.append([x,y])
        if (x <= 5) and (x >= 2) and (y == 5):  # Wall 2
            obsCord.append([x,y])
        if (x <= 3) and (x >= 2) and (y == 7):  # Wall 3
            obsCord.append([x,y])
        if (x <= 14) and (x >= 9) and (y == 2):  # Wall 4
            obsCord.append([x,y])
        if (x <= 11) and (x >= 9) and (y == 4):  # Wall 5
            obsCord.append([x,y])
        if (x <= 15) and (x >= 7) and (y == 7):  # Wall 6
            obsCord.append([x,y])
        if (x <= 16) and (x >= 13) and (y == 5):  # Wall 7
            obsCord.append([x,y])

    return(mapCord,obsCord)
            
        
## ------------------------------------------------------------------------------------------
#                                       Main Function
## ------------------------------------------------------------------------------------------

if __name__== "__main__":
    
    s = [16,1] # Start State
    g = [3,6] # Goal State

    # Map Size is set as:
    mapWidth = 16
    mapHeight = 8  
      
    # Build a Map
    mapCord, obsCord = buildMap(mapHeight, mapWidth)
    
    # checks if inputs are Valid
    if s not in mapCord:
        print("Start Node outside Map")
    elif g not in mapCord:
        print("Goal Node outside Map")
    elif s in obsCord:
        print("Start Node inside Map")
    elif g in obsCord:
        print("Goal Node inside Map")
    elif s == g: # Check if start node is goal node
        print("Start node is Goal Node!!")
    else: 
        print("Implementing A* Search")
        print("===============================================================================================")
        aStar(s, g, obsCord)
    
## ------------------------------------------------------------------------------------------
#                                Display --> Forward and Backward Path
## ------------------------------------------------------------------------------------------

end_time = time.time()

print("===============================================================================================")
print("Time to Find Solution Path", round((end_time - start_time), 3), "seconds")
print("===============================================================================================")

print('\n')
