#! /usr/bin/env python

import numpy as np
import sys
import cv2
from matplotlib import pyplot as plt

index = 0
end = []

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def Astar(maze, start, end, fig):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    startNode = Node(None, start)
    startNode.h =  startNode.f = 0
    startNode.g =  startNode.f = 0
    endNode = Node(None, end)
    endNode.h = endNode.f = 0
    endNode.g = endNode.f = 0

    # Initialize both open and closed list
    openList = []
    closedList = []

    # Add the start node
    openList.append(startNode)
    
    n = 1
    # Loop until you find the end
    while len(openList) > 0:

        # Get the current node
        currentNode = openList[0]
        current_index = 0
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                currentNode = item
                current_index = index

        # Pop current off open list, add to closed list
        openList.pop(current_index)
        closedList.append(currentNode)

        # Found the goal
        if currentNode == endNode:
            path = []
            waypoints = []
            current = currentNode
            while current is not None:
                current_wp = (0.05 * (current.position[1] - 232), 0.05 * (70 - current.position[0]))
                path.append(current.position)
                waypoints.append(current_wp)
                current = current.parent
            
            # mm = genMaze(maze.copy(), start, end, openList, closedList, path[::-1])
            # pltMaze(mm, fig)
            # waypoints = waypoints[::-1]
            # print(waypoints)
            return (path[::-1], waypoints[::-1]) # Return reversed path

        # Generate children
        children = []
        for x, y in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            nodePosition = (currentNode.position[0] + x, currentNode.position[1] + y)

            # Make sure within range
            if nodePosition[0] > (len(maze) - 1) or nodePosition[0] < 0 or nodePosition[1] > (len(maze[len(maze)-1]) -1) or nodePosition[1] < 0:
                continue
            # import pdb; pdb.set_trace()
            # Make sure walkable terrain
            if maze[nodePosition[0]][nodePosition[1]] != 0:
                continue
            # if maze[nodePosition[0]][nodePosition[1]] != 0:
            #     continue

            # Create new node
            newNode = Node(currentNode, nodePosition)

            # Append
            children.append(newNode)

        # Loop through children
        for child in children:

            # Child is on the closed list
            b = False
            for closedChild in closedList:
                if child == closedChild:
                    b = True #continue
                    break
            if b:
                continue
            
            # Create the f and g
#            child.g = currentNode.g + 1
            child.g = ((child.position[0] - endNode.position[0]) ** 2) + ((child.position[1] - endNode.position[1]) ** 2)
            child.h = ((child.position[0] - startNode.position[0]) ** 2) + ((child.position[1] - startNode.position[1]) ** 2)
            child.f = child.h + child.g

            # Child is already in the open list
            b = False
            for openNode in openList:
                if child == openNode and child.h >= openNode.h:
                    b = True #continue
                    break
            if b:
                continue

            # Add the child to the open list
            openList.append(child)
            
        # if (n % 2) == 1 :
        #     mm = genMaze(maze.copy(), start, end, openList, closedList)
        #     pltMaze(mm, fig)
            
        n = n + 1

def pltMaze(maze, fig):
    fig.clf()
    ax = fig.add_subplot(1, 1, 1)  # create an axes object in the figure
    
    ax.matshow(maze, cmap=plt.cm.bone)
    for (i, j), z in np.ndenumerate(maze):
        plt.text(j, i, '{:0.0f}'.format(z), ha='center', va='center')

 
    plt.draw()
    plt.pause(0.0001)
def click_event(event,x,y,flags,param):
    global start, end, index
    # if event==cv2.EVENT_LBUTTONDOWN:
    #     print("INIT")
    #     start=(y,x)
    if event==cv2.EVENT_RBUTTONDOWN:
        print('FIN')
        point = (y, x)
        end.append(point)
        index = index + 1

if __name__ == "__main__":
    image=cv2.imread('/home/user/catkin_ws/src/ros_wall_follower/Maps/berlin_5cm.png')
    cv2.imshow('ferdinand',image)
    img=cv2.resize(image,None,fx=1.0,fy=1.0)
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, thresh=cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)
    norm=np.array(thresh/254)
    cv2.imshow("popo",gray)
    np.set_printoptions(threshold=sys.maxsize)
    #print(gray)
    norm=np.array(thresh/255)

    cv2.setMouseCallback("popo",click_event)
    # start = ()
    # end = ()
    start = (70, 232)
    # end = (75, 240)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    wpCSV = []
    index = index - 1
    num = 0
    while (num <= index):
        [pathAstar, wpAstar] = Astar(norm, start, end[num], None)
        start = end[num]
        num += 1
        wpCSV += wpAstar[:-1]
        print(pathAstar)
        print(wpAstar)

    np.savetxt('./scripts/odom_data_A*.csv',wpCSV, delimiter = ",")
    # fig = plt.figure(figsize = (20, 20))  # create a figure object
    # normCopy = norm.copy()
    # normCopy[start] = 2
    # normCopy[end] = 2
    # pltMaze(normCopy, fig)