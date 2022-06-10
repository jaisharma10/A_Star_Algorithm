## ------------------------------------------------------------------------------------------
#                                  Depth First Search [Empty Map]
## ------------------------------------------------------------------------------------------

'''
Author: Jai Sharma
Task: implement A* algorithm on an empty 10 x 10 map between a given start and goal node
        
'''

## ------------------------------------------------------------------------------------------
#                                        Import Libraries
## ------------------------------------------------------------------------------------------

import time
import copy
from collections import deque
import pygame
import sys
import numpy as np
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
        if col < 10:  # node above exists
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
        if row < 10:  
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
        if col < 10 and row < 10:  
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row + 1, col + 1
            newNode.c2c = round(self.c2c + 1.4, 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       # Up not possible
   
    def moveDownRight(self, pos, goal): 
        row, col = pos[0], pos[1]
        if col > 1 and row < 10:  # node above exists
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row + 1, col - 1
            newNode.c2c = round(self.c2c + 1.4, 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode)    
        else:
            return(False)       
    
    def moveUpLeft(self, pos, goal): 
        row, col = pos[0], pos[1]
        if row > 1 and col < 10: 
            # Initialize New Node
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row - 1, col + 1
            newNode.c2c = round(self.c2c + 1.4, 3)
            newNode.c2g = self.manhatten(newNode.state, goal)
            newNode.cost = round(newNode.c2c + newNode.c2g,3)
            return(newNode) 
        else:
            return(False)       
    
    def moveDownLeft(self, pos, goal): 
        row, col = pos[0], pos[1]
        if row > 1 and col > 1: 
            # Initialize New Node
            print("moveDownLeft moveDownLeft moveDownLeft")
            newNode = Node(copy.deepcopy(self.state), Node(self.state, self.parent, self.c2c,0,0),0,0,0)  
            newNode.state[0], newNode.state[1]  = row - 1, col - 1
            newNode.c2c = round(self.c2c + 1.4, 3)
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

def aStar(s, g):
    
    pygame.init()
    magf = 50 # magnification factor
    screen = pygame.display.set_mode(((13)*magf, (13)*magf))
    screen.fill((30,30,30))
    
    initialDistance = round(distance.euclidean(s,g),3)
    startNode = Node(s, None, 0, initialDistance, initialDistance)
    goalNode = Node(g, None, float('inf'), 0, float('inf'))

    queue = []                    # all neighbour states to explore
    visited = []                  # all visited lists fall here
    queue.append(startNode)       # add start node to queue
    visited.append(startNode.state)
    
    while queue != []:
        print("queue",queue)
        queue.sort(key = lambda x: x.cost)                # sort queue based on c2c
        currentNode = queue.pop(0)                        # pop node with lowest cost 
        print("currentNode -->",currentNode)
        # print("c2c:",currentNode.c2c)
        # print("c2g:",currentNode.c2g)
        # print("total:",currentNode.cost)


        print("==================================================================================================")
        pygame.draw.circle(screen, (0,128,0), (magf*(1 + goalNode.state[0]), 12*magf-magf*goalNode.state[1]), 16)   # Goal Node
        pygame.draw.circle(screen, (255,0,0), (magf*(1 + startNode.state[0]), 12*magf-magf*startNode.state[1]), 16) # Start Node
        pygame.draw.circle(screen, (255,255,255), (magf*(1 + currentNode.state[0]), 12*magf-magf*currentNode.state[1]), 9)   # Current Node
        pygame.display.update()
        time.sleep(0.5)
        
        # Case 1 --> Goal Reached
        if currentNode.state == goalNode.state:  # check if goal state reached
            print("Goal Reached !") 
            backTrackList = backtrack(currentNode, startNode)  # backtrack list is goal to start
            reversed_backTrackList = backTrackList[::-1] # reversed --> list is start to goal
            prev = reversed_backTrackList[0]
            print("backTrackList", backTrackList)
            print("Cost to reach Goal Node -->", currentNode.cost)
            print("Eucilidean Distance-->", initialDistance)
            print("cost to come:",currentNode.c2c)
            for route in reversed_backTrackList:   # visualize the search algorithm
                pygame.draw.circle(screen, (0,0,250), (magf*(1 + route[0]), 12*magf-magf*route[1]), 7)   # Current Node     
                pygame.draw.line(screen, (255, 255, 0), (magf*(1 + route[0]), 12*magf-magf*route[1]), (magf*(1 + prev[0]), 12*magf-magf*prev[1]),5)
                pygame.draw.circle(screen, (0,0,250), (magf*(1 + prev[0]), 12*magf-magf*prev[1]), 7)   # Current Node     
                pygame.display.update()
                prev = route
            time.sleep(15)
            break

        # Case 2: goal not reached, evaluate neighbours to the queue 
        else: 
            Neighbours = currentNode.getNeighbours(currentNode.state, goalNode.state)  # get neighbours of current node
            # print("Neighbours", Neighbours) 
            for child in Neighbours:

                # Case2A: previosly explored, update if needed
                if child.state in visited:
                    # print("========B=======")
                    if child.parent != currentNode.state:
                        parentNode = child.parent     
                        if child.cost > parentNode.cost + 1: # update node if needed
                            child.cost = parentNode.cost + 1
                     
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
#                                  Backtracking Function
## ------------------------------------------------------------------------------------------

def backtrack(current, start):
    backtrackList = [current.state]   # new list to collect backtracked list
    while(current.state != start.state):
        current = current.parent
        backtrackList.append(current.state)
    return(backtrackList)
        
## ------------------------------------------------------------------------------------------
#                                       Main Function
## ------------------------------------------------------------------------------------------

if __name__== "__main__":
    
    s = [6,6] # Start State
    g = [10,10] # Goal State

    # Map Size is set as:
    mapWidth = 10
    mapHeight = 10
    
    # Build a Map
    mapCord = []
    for x in range(1, mapWidth + 1, 1):
        for y in range(1, mapHeight + 1,1):
            mapCord.append([x,y])

    # checks if inputs are Valid
    if s not in mapCord:
        print("Start Node outside Map")
    elif g not in mapCord:
        print("Goal Node outside Map")
    elif s == g: # Check if start node is goal node
        print("Start node is Goal Node!!")
    else: 
        print("Implementing A* Search")
        print("===============================================================================================")
        aStar(s, g)
    
## ------------------------------------------------------------------------------------------
#                                Display --> Forward and Backward Path
## ------------------------------------------------------------------------------------------

end_time = time.time()

print("===============================================================================================")
print("Time to Find Solution Path", round((end_time - start_time), 3), "seconds")
print("===============================================================================================")

print('\n')
