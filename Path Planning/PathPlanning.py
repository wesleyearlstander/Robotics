import numpy as np
import math
import random
import sys
import shortest_path

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.links = []

    def distance(self, node):
        x = math.pow(node.x-self.x,2)
        y = math.pow(node.y-self.y,2)
        return math.sqrt(x+y)


class Obstacle:
    def __init__(self, left, top, right, bottom):
        self.left = left
        self.top = top
        self.right = right
        self.bottom = bottom

    def collided(self, node):
        if self.withinWidth(node) and self.withinHeight(node):
            return True
        return False

    def withinHeight(self, node):
        if node.y >= float(self.bottom) and node.y <= float(self.top):
            return True
        return False

    def withinWidth (self, node):
        if node.x >= float(self.left) and node.x <= float(self.right):
            return True
        return False

    def withinWidthH (self, x):
        if x >= float(self.left) and x <= float(self.right):
            return True
        return False

    def withinHeightH (self, y):
        if y >= float(self.bottom) and y <= float(self.top):
            return True
        return False


    def blocked(self, node, node2):
        xD = abs(node2.x - node.x)
        yD = abs(node2.y - node.y)

        slope = yD/xD
        intercept = node.y - slope*node.x

        leftY = float(self.left) * slope + intercept
        rightY = float(self.right) * slope + intercept

        topX = (float(self.top) - intercept)/slope
        bottomX = (float(self.bottom) - intercept)/slope

        if (self.withinHeightH(leftY) or self.withinHeightH(rightY) or self.withinWidthH(topX) or self.withinWidthH(bottomX)):
            return True
        return False


class Map:
    def __init__(self, width, height, start, end, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.milestones = []
        self.startNode = start
        self.endNode = end

    def generateMilestones(self, amount):
        i = 0
        while i < amount:
            x = random.uniform(0, self.width)
            y = random.uniform(0, self.height)
            node = Node(x,y)
            col = False
            for j in range(len(self.obstacles)):
                if (self.obstacles[j].collided(node)):
                    col = True
            if not col:
                self.milestones.append(node)
                i+=1

    def closestLinks(self, closest):
        nodeDelete = []
        self.milestones.append(self.startNode)
        self.milestones.append(self.endNode)
        for i in range(len(self.milestones)):
            node = self.milestones[i]
            links = []
            for j in range(len(self.milestones)):
                if i != j:
                    if len(links) < closest:
                        links.append(self.milestones[j])
                    else:
                        break
            for k in range(len(links)):
                for j in range(len(self.milestones)):
                    if i != j:
                        if node.distance(links[k]) > node.distance(self.milestones[j]) and not self.milestones[j] in links:
                            links[k] = self.milestones[j]
            delete = []
            for j in range(closest):
                for k in range(len(self.obstacles)):
                    if self.obstacles[k].blocked(node, links[j]):
                        delete.append(j)
                        break
            count = 0
            for j in range(len(delete)):
                del links[delete[j] - count]
                count+=1
            node.links = links
            if len(links) < 2 and node != self.startNode and node != self.endNode:
                nodeDelete.append(i)
        count = 0
        for i in range(len(nodeDelete)):
            del self.milestones[nodeDelete[i] - count]
            count+=1


def ReadFile():
    f = open("input.txt", "r")
    count = 0
    startNode = None
    endNode = None
    obstacles = []
    for x in f:
        coords = x.split(';')
        if len(coords) > 1:
            start = coords[0].split(',')
            end = coords[1].split(',')
            if count == 0:
                startNode = Node(float(start[0]), float(start[1]))
                endNode = Node(float(end[0]), float(end[1].rstrip("\n")))
            else :
                obstacles.append(Obstacle(float(start[0]), float(end[1].rstrip("\n")), float(end[0]), float(start[1])))
            count+=1
    f.close()
    return [startNode, endNode, obstacles]

def main():
    input = ReadFile()
    map = Map(100,100,input[0],input[1],input[2])
    map.generateMilestones(250)
    map.closestLinks(10)
#     for i in range(len(map.milestones)):
#         print ((map.milestones[i].links))
        #for j in range(len(map.milestones[i].links)):
            #print map.milestones[i].distance(map.milestones[i].links[j])
    path, curr = shortest_path.a_star(map.startNode, map.endNode)
    if curr.x != map.endNode.x:
        main()
        return
    shortest_path.print_path(path, curr)
if __name__ == "__main__":
    main()
