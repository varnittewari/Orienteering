import numpy as np
import queue
from PIL import Image

import math

class Node(object):
    x = 0
    y = 0
    f = 0
    g = 0
    h = 0
    elevation = 0
    parent = None

    def __init__(self,x,y,elevation):
        self.x = x
        self.y = y
        self.elevation = elevation

    def setParent(self,parent):
        self.parent=parent

    def setHeur(self,f,g,h):
        self.f = f
        self.g = g
        self.h = h

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getF(self):
        return self.f

    def getG(self):
        return self.g

    def getH(self):
        return self.h

    def getParent(self):
        return self.parent

    def getElevation(self):
        return self.elevation

    def equals(self,n):
        if n.getX() == self.x and n.getY() == self.y:
            return True
        return False

def winter(graph,elevations):
    edge = []
    for i in range(395):
        for j in range(500):
            r2, g2, b2 = graph.getpixel((i, j))
            if r2 == 0 and g2 == 0 and b2 == 255:
                n = Node(i,j,elevations[i][j])
                for next in getNeighbors(n,elevations):
                    x = next.getX()
                    y = next.getY()
                    r1,g1,b1 = graph.getpixel((x, y))
                    if r1 == 248 and g1 == 148 and b1 == 18:  # open land
                        edge.append(n)
                    elif r1 == 255 and g1 == 192 and b1 == 0:  # rough meadow
                        edge.append(n)
                    elif r1 == 255 and g1 == 255 and b1 == 255:  # easy movement forest
                        edge.append(n)
                    elif r1 == 2 and g1 == 208 and b1 == 60:  # slow run forest
                        edge.append(n)
                    elif r1 == 2 and g1 == 136 and b1 == 40:  # walk forest
                        edge.append(n)
                    elif r1 == 5 and g1 == 73 and b1 == 24:  # impassible vegetation
                        edge.append(n)
                    elif r1 == 71 and g1 == 51 and b1 == 3:  # paved road
                        edge.append(n)
                    elif r1 == 0 and g1 == 0 and b1 == 0:  # footpath
                        edge.append(n)

def bfs(graph,src,elevations):
    res =[]
    q = []
    visited = []
    frontier = 0
    for n in src:
        q.append(n)
        visited.append(n)
        while frontier<7:
            q.remove(src)
            for neig in getNeighbors(n,elevations):
                x = neig.getX()
                y = neig.getY()
                r3,g3,b3 = graph.getpixel((x,y))
                if r3 == 0 and g3 == 0 and b3 == 255:
                    if not contains(visited,neig):
                        res.append(neig)
                        q.append(neig)
                        visited.append(neig)
        frontier+=1

#def spring(img):



def getDistance(a,b,c,d,elevations):
    x_c = 0
    y_c = 0
    if a!=c:
        x_c = 7.55
    elif b!=d:
        y_c = 10.29
    z = float(elevations[a][b])-float(elevations[c][d])

    return math.sqrt(x_c**2 + y_c**2 + z**2)

def getCost(curr,next,graph,elevations):
    """how to find the cost of one move"""
    a1 = next.getX()
    b1 = next.getY()
    a2 = curr.getX()
    b2 = curr.getY()
    dist = getDistance(a1, b1, a2, b2,elevations)
    cost = dist
    r1, g1, b1 = graph.getpixel((a1, b1))
    if r1 == 248 and g1 == 148 and b1 == 18: #open land
        #dist = dist+40
        cost = dist/40
    elif r1 == 255 and g1 == 192 and b1 == 0:  #rough meadow
        #dist = dist + 70
        cost = dist /18
    elif r1 == 255 and g1 == 255 and b1 == 255:  #easy movement forest
        #dist = dist + 60
        cost = dist /15
    elif r1 == 2 and g1 == 208 and b1 == 60:   #slow run forest
        #dist = dist + 80
        cost = dist /10
    elif r1 == 2 and g1 == 136 and b1 == 40:    #walk forest
        #dist = dist + 100
        cost = dist /6
    elif r1 == 5 and g1 == 73 and b1 == 24:     #impassible vegetation
        #dist = dist + 1000
        cost = dist /0.001    #divided by a float
    elif r1 == 0 and g1 == 0 and b1 == 255:     #lake/swamp/marsh
        #dist = dist + 4000
        cost = dist /0.00001
    elif r1 == 71 and g1 == 51 and b1 == 3:     #paved road
        #dist = dist + 40
        cost = dist /200
    elif r1 == 0 and g1 == 0 and b1 == 0:       #footpath
        #dist = dist + 40
        cost = dist /150
    elif r1 == 201 and g1 == 0 and b1 == 101:
        cost = 1000000
        #dist = dist + 40000

    print( r1,g1,b1,cost)
    return cost


def getNeighbors(curr,elevations):
    neighbors = []
    x = curr.getX()
    y = curr.getY()

    if (x==0):
        if y==0:
            neighbors.append(Node(x + 1, y, elevations[x + 1][y]))
            neighbors.append(Node(x, y + 1, elevations[x][y + 1]))
        elif y==500:
            neighbors.append(Node(x + 1, y, elevations[x + 1][y]))
            neighbors.append(Node(x, y - 1, elevations[x][y - 1]))
        else:
            neighbors.append(Node(x + 1, y, elevations[x + 1][y]))
            neighbors.append(Node(x, y - 1, elevations[x][y - 1]))
            neighbors.append(Node(x, y + 1, elevations[x][y + 1]))

    elif x==395:
        if y==0:
            neighbors.append(Node(x - 1, y, elevations[x - 1][y]))
            neighbors.append(Node(x, y + 1, elevations[x][y + 1]))
        elif y==500:
            neighbors.append(Node(x - 1, y, elevations[x - 1][y]))
            neighbors.append(Node(x, y - 1, elevations[x][y - 1]))
        else:
            neighbors.append(Node(x - 1, y, elevations[x - 1][y]))
            neighbors.append(Node(x, y - 1, elevations[x][y - 1]))
            neighbors.append(Node(x, y + 1, elevations[x][y + 1]))

    elif y==0:
        neighbors.append(Node(x - 1, y, elevations[x - 1][y]))
        neighbors.append(Node(x + 1, y, elevations[x + 1][y]))
        neighbors.append(Node(x, y + 1, elevations[x][y + 1]))

    elif y==500:
        neighbors.append(Node(x - 1, y, elevations[x - 1][y]))
        neighbors.append(Node(x + 1, y, elevations[x + 1][y]))
        neighbors.append(Node(x, y - 1, elevations[x][y - 1]))

    else:
        neighbors.append(Node(x-1,y,elevations[x-1][y]))
        neighbors.append(Node(x+1,y,elevations[x+1][y]))
        neighbors.append(Node(x,y-1,elevations[x][y-1]))
        neighbors.append(Node(x,y+1,elevations[x][y+1]))

    return neighbors

def heuristic(goal,next):
    #print(goal.getElevation())
    #print(next.getElevation())
    x_coor = (next.getX() - goal.getX())*10.29
    y_coor = (next.getY() - goal.getY())*7.55
    z_coor = float(next.getElevation()) - float(goal.getElevation())
    h = math.sqrt(x_coor**2 + y_coor**2 + z_coor**2)
    return h

def minF(lst):
    min = 10000000
    min_node= None
    for node in lst:
        if node.getF() <= min:
            min = node.getF()
            min_node = node

    return min_node

def checkIfNodePresent(node,lst):
    for n in lst:
        if n.equals(node):
            if n.getF() < node.getF():
                return True
    return False


def changePix(res,graph):
    rgb_im = graph.load()
    res = res[:len(res)-1]
    for r in res:
        x = r.getX()
        y = r.getY()
        rgb_im[x,y] = (220,20,60)

    return graph

def goalFound(curr,open_lst):
    res_path = []
    tot_dist = curr.getF()
    """
    for n in open_lst:
        if n.equals(curr):
            n.setParent(curr.getParent())
    """
    new = curr
    res_path.append(new)
    while (new != None):
        res_path.append(new.getParent())
        new = new.getParent()
    return (res_path,tot_dist)

def contains(lst,n):
    for new in lst:
        if new.equals(n):
            return True
    return False

def containsA(lst,n):
    for new in lst:
        if new.equals(n) and n.getG() < new.getG():
            return True
    return False

def astar(graph, start, goal,elevations):
    res_path = []
    tot_dist = 0
    open_lst=[]
    closed_lst=[]
    open_lst.append(start)

    h1 = heuristic(goal,start)
    start.setHeur(h1,0,h1)
    start.setParent(None)

    while len(open_lst)!=0:
        curr = minF(open_lst)
        open_lst.remove(curr)

        if curr.equals(goal):
            res_path,tot_dist = goalFound(curr,open_lst)
            break

        closed_lst.append(curr)

        for next in getNeighbors(curr,elevations):
            next.setParent(curr)
            print(next.getX(),next.getY())

            if not contains(closed_lst,next):
                g = curr.getG() + getCost(curr,next, graph,elevations)  # int getting added by a float
                h = heuristic(goal, next)
                f = g + h
                next.setHeur(f, g, h)
                if contains(open_lst,next):
                    #next.setHeur(f, g, h)
                    for new in open_lst:  #both nodes have same x and y but different f,g,h?
                        if containsA(open_lst,new):
                            new.setHeur(new.getF(),next.getG(),new.getH())
                            new.setParent(next.getParent())
                else:
                    open_lst.append(next)

    print("Total Distance = "+ str(tot_dist))
    rGraph = changePix(res_path,graph)
    return rGraph

def main():
    graph = Image.open('terrain.png')
    graph = graph.convert('RGB')

    my_file = open("elevation_file.txt")
    elevations = []
    for line in my_file:
        line = line.split()
        line = line[:len(line) - 5]
        elevations.append(line)

    path_fil= open("brown.txt")
    path = []
    for line in path_fil:
        line= line.split()
        a = int(line[0])
        b = int(line[1])
        path_node = Node(a,b, elevations[a][b])
        path.append(path_node)
        #print(a,b)
        #print(elevations[a][b])

    rGraph = graph
    for i in range(len(path)):
        if i==len(path)-1:
            break
        else:
            rGraph = astar(graph,path[i],path[i+1],elevations)
            #rGraph.show()

    #print(float(elevations[a][b]))
    #astar(graph,Node(252,324,'2.3853529e+02'),Node(243,327,'2.1611823e+02'),elevations)
    rGraph.show()

graph = Image.open('terrain.png')
rgb_im = graph.convert('RGB')
r1, g1, b1 = rgb_im.getpixel((243, 327))
#print(r1,g1,b1)
main()