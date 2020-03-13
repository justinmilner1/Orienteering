"""
Intro to AI Lab 1
@author: Justin Milner
"""
import sys
import math
from PIL import Image

#Constants: Color values for terrain
OPEN_LAND = (248, 148, 18)
ROUGH_MEADOW = (255, 192, 0)
EASY_MOVEMENT_FOREST = (255, 255, 255)
SLOW_RUN_FOREST = (2, 208, 60)
WALK_FOREST = (2, 136, 40)
IMPASSIBLE_VEGETATION = (5, 73, 24)
LAKE_SWAMP_MARSH = (0, 0, 255)
PAVED_ROAD = (71,51,3)
FOOT_PATH = (0, 0, 0)
OUT_OF_BOUNDS = (205, 0, 101)
ICE = (135, 206, 250)
MUD = (143, 116, 63)
LEAVES = (190, 34, 34)

#Constants: Borders
X_LOWER = 0
Y_LOWER = 0
X_UPPER = 395
Y_UPPER = 500

#Constants: Scaling
LONGITUDE_SCALING = 7.55
LATITUDE_SCALING = 10.29

class Node:
    """
    Node representing a single pixel position
    """
    def __init__(self, xCoord, yCoord, parent, elevation, fScore, gScore , depth=0):
        self.xCoord = xCoord
        self.yCoord = yCoord
        self.parent = parent
        self.elevation = elevation
        self.fScore = fScore
        self.gScore = gScore
        self.depth = depth

elevations = []
imageString = sys.argv[1]
terrainImage = Image.open(imageString)
terrainImage = terrainImage.convert("RGB")


def main():
    # taking inputs
    for line in open(sys.argv[2]):
        line = line.strip().split()
        elevations.append(line)
    startDest = sys.argv[3]
    season = sys.argv[4]
    output = sys.argv[5]
    chooseSeason(season)  # applying seasonal changes, initiates graph

    path = []
    lineNumber = 0

    for line in open(startDest):
        line = line.strip().split()
        line[0] = int(line[0])
        line[1] = int(line[1])
        if lineNumber == 0:
            lineNumber += 1
            start = Node(line[0], line[1], None, elevations[line[1]][line[0]], 0, 0)
        else:
            end = Node(line[0], line[1], None, elevations[line[1]][line[0]], 0, 0)
            path += path + aStar(start, end) #calling astar
            start = end

    bestDist = 0
    first = path[0]
    for node in path:
        second = node

        bestDist += math.sqrt( planeDistance(first.xCoord, first.yCoord, second.xCoord, second.yCoord) +
                               elevationDistance(elevations[first.yCoord][first.xCoord], elevations[second.yCoord][second.xCoord]) )
        terrainImage.putpixel((node.xCoord, node.yCoord), (156, 41, 241))
        first = node
    print("Total distance:", bestDist, "meters")
    terrainImage.save(output)



def aStar(start, end):
    """
       finds shortest path between given points
       :param start node
       :param end node
       :return: path of nodes start to finish
       """
    openList = set()
    openList.add(start)

    closedList = set()
    start.gScore = 0
    start.fScore = start.gScore + math.sqrt( planeDistance(start.xCoord, start.yCoord, end.xCoord, end.yCoord) + elevationDistance(start.elevation, end.elevation) )
    while len(openList) > 0:
        selectedNode = None
        lowestScore = 0
        for node in openList:
            if selectedNode == None or lowestScore > node.fScore:
                selectedNode = node
                lowestScore = node.fScore

        if selectedNode.xCoord == end.xCoord and selectedNode.yCoord == end.yCoord:
            path = []
            path.append(selectedNode)
            while selectedNode.parent:
                selectedNode = selectedNode.parent
                path.append(selectedNode)
            return path

        openList.remove(selectedNode)
        closedList.add(selectedNode)
        neighbors = getNodeNeighbors(selectedNode)
        for neighbor in neighbors:
            if inSet(neighbor, closedList) == False:
                neighbor.fScore = neighbor.gScore + math.sqrt( planeDistance(neighbor.xCoord, neighbor.yCoord, end.xCoord, end.yCoord)
                                                               + elevationDistance(neighbor.elevation, end.elevation) )
                openNeighbor = inSet(neighbor, openList)
                if openNeighbor == False:
                    openList.add(neighbor)
                else:
                    if neighbor.gScore < openNeighbor.gScore:
                        openNeighbor.gScore = neighbor.gScore
                        openNeighbor.parent = neighbor.parent
    return False


def planeDistance(startX, startY, endX, endY):
    """
    Gives the planar distance betweeen two points (heuristic)
    :param startX:
    :param startY:
    :param endX:
    :param endY:
    :return:
    """
    return math.pow( (startX * LATITUDE_SCALING) - (endX * LATITUDE_SCALING), 2) + \
           math.pow((startY * LONGITUDE_SCALING) - (endY * LONGITUDE_SCALING), 2)

def elevationDistance(startE, endE):
    """
    Gives the elevation change between two points (heuristic)
    :param startE:
    :param endE:
    :return:
    """
    return math.pow(float(startE) - float(endE), 2 )


def inSet(search, set):
    """
    if search is in the set, returns search, otherwise returns False
    :param search(node)
    :param set
    """
    for node in set:
        if search.xCoord == node.xCoord and search.yCoord == node.yCoord:
            return node

    return False


def getGScore(initial, finish):
    """
    Based on type of terrain calculates a g score
    :param start: initial node
    :param end: end node
    :return: g score
    """

    dist = math.sqrt( planeDistance(initial.xCoord, initial.yCoord, finish[0], finish[1]))
    if finish[2] == OPEN_LAND:
        gScore = 1
        return initial.gScore + (gScore * dist)
    if finish[2] == ROUGH_MEADOW:
        gScore = 1.2
        return initial.gScore + (gScore * dist)
    if finish[2] == EASY_MOVEMENT_FOREST:
        gScore = 1.5
        return initial.gScore + (gScore * dist)
    if finish[2] == LEAVES:
        gScore = 1.5
        return initial.gScore + (gScore * dist)
    if finish[2] == SLOW_RUN_FOREST:
        gScore = 3
        return initial.gScore + (gScore * dist)
    if finish[2] == WALK_FOREST:
        gScore = 2
        return initial.gScore + (gScore * dist)
    if finish[2] == IMPASSIBLE_VEGETATION:
        gScore = 5
        return initial.gScore + (gScore * dist)
    if finish[2] == LAKE_SWAMP_MARSH:
        gScore = 8
        return initial.gScore + (gScore * dist)
    if finish[2] == PAVED_ROAD:
        gScore = 0.8
        return initial.gScore + (gScore * dist)
    if finish[2] == FOOT_PATH:
        gScore = 0.9
        return initial.gScore + (gScore * dist)
    if finish[2] == ICE:
        gScore = 3
        return initial.gScore + (gScore * dist)
    if finish[2] == MUD:
        gScore = 7
        return initial.gScore + (gScore * dist)


def getNodeNeighbors(currentNode):
    """
    Returns the node-form neighbors of a given point
    :param node: given node
    :return: set of neighbors
    """
    x = currentNode.xCoord
    y = currentNode.yCoord
    neighbors = set()

    if x - 1 > X_LOWER:
        if terrainImage.getpixel((x - 1, y)) != OUT_OF_BOUNDS:
            neighbors.add(Node(x - 1, y, currentNode, elevations[y][x - 1], 0, getGScore(currentNode, (x - 1, y, terrainImage.getpixel((x - 1, y))))))
    if x + 1 < X_UPPER:
        if terrainImage.getpixel((x + 1, y)) != OUT_OF_BOUNDS:
            neighbors.add(Node(x + 1, y, currentNode, elevations[y][x + 1], 0, getGScore(currentNode, (x + 1, y, terrainImage.getpixel((x + 1, y))))))
    if y - 1 > Y_LOWER:
        if terrainImage.getpixel((x, y - 1)) != OUT_OF_BOUNDS:
            neighbors.add(Node(x, y - 1, currentNode, elevations[y - 1][x], 0, getGScore(currentNode, (x, y - 1, terrainImage.getpixel((x, y - 1))))))
    if y + 1 < Y_UPPER:
        if terrainImage.getpixel((x, y + 1)) != OUT_OF_BOUNDS:
            neighbors.add( Node(x, y + 1, currentNode, elevations[y + 1][x], 0, getGScore(currentNode, (x, y + 1, terrainImage.getpixel((x, y + 1))))))
    return neighbors



def getPositionNeighbors(x, y):
    """
    Returns coordinate-form neighbors of a given point
    :return: neighbors
    """
    coords = set()
    if x > X_LOWER:
        coords.add((x - 1, y))
    if x < X_UPPER -1 :
        coords.add((x + 1, y))
    if y > Y_LOWER:
        coords.add((x, y - 1))
    if y < Y_UPPER - 1:
        coords.add((x, y + 1))

    return coords


def chooseSeason(season):
    """
    Selects terrain based on season input
    :param string season
    # """
    nodes = []
    wint = False
    spr = False
    for row in range(terrainImage.size[0]):
        for col in range(terrainImage.size[1]):
            if season.lower() == "spring":
                spr = True;
                if terrainImage.getpixel((row, col)) == LAKE_SWAMP_MARSH:
                    neighbor = getPositionNeighbors(row, col)
                    currentElevation = float(elevations[col][row])
                    for n in neighbor:
                        neighbor_elevation = float(elevations[n[1]][n[0]])
                        changeInElevation = neighbor_elevation - currentElevation
                        if terrainImage.getpixel((n[0], n[1])) not in [LAKE_SWAMP_MARSH, OUT_OF_BOUNDS] and changeInElevation <= 1:
                            nodes.append(Node(n[0], n[1], 0, 0, n[0], n[1], 1))

            elif season.lower() == "winter":
                wint = True;
                if terrainImage.getpixel((row, col)) not in [LAKE_SWAMP_MARSH, OUT_OF_BOUNDS]:
                    neighbor = getPositionNeighbors(row, col)
                    for n in neighbor:
                        if terrainImage.getpixel((n[0], n[1])) == LAKE_SWAMP_MARSH:
                            nodes.append(Node(n[0], n[1], 0, 0, 0, 0, 1))

            elif season.lower() == 'fall':
                if terrainImage.getpixel((row, col)) == EASY_MOVEMENT_FOREST:
                    neighbor = getPositionNeighbors(row, col)
                    for n in neighbor:
                        if terrainImage.getpixel((n[0], n[1])) == FOOT_PATH:
                            terrainImage.putpixel((n[0], n[1]), LEAVES)
    #starting bfs's
    if wint:
        frontier = set()
        for position in nodes:
            frontier.add((position.xCoord, position.yCoord))

        while nodes:
            s = nodes.pop(0)
            terrainImage.putpixel((s.xCoord, s.yCoord), ICE)
            if s.depth != 7:  # nodes within 7 pixels from water
                potential_neighbors = getPositionNeighbors(s.xCoord, s.yCoord)
                for neighbor in potential_neighbors:
                    if terrainImage.getpixel((neighbor[0], neighbor[1])) == LAKE_SWAMP_MARSH and (
                    neighbor[0], neighbor[1]) not in frontier:
                        frontier.add((neighbor[0], neighbor[1]))
                        node = Node(neighbor[0], neighbor[1], 0, 0, 0, 0, s.depth + 1)
                        nodes.append(node)
    if spr:
        frontier = set()
        for position in nodes:
            frontier.add((position.xCoord, position.yCoord))

        while nodes:
            s = nodes.pop(0)
            terrainImage.putpixel((s.xCoord, s.yCoord), MUD)
            start_elevation = float(elevations[s.gScore][s.fScore])
            if s.depth != 15:  # nodes within 15 pixels from water
                neighbors = getPositionNeighbors(s.xCoord, s.yCoord)
                for neighbor in neighbors:
                    neighborElevation = float(elevations[neighbor[1]][neighbor[0]])
                    changeInElevation = neighborElevation - start_elevation

                    if terrainImage.getpixel((neighbor[0], neighbor[1])) not in [LAKE_SWAMP_MARSH, MUD,
                                                                                 OUT_OF_BOUNDS] and (
                    neighbor[0], neighbor[1]) not in frontier and changeInElevation <= 1:
                        frontier.add((neighbor[0], neighbor[1]))
                        node = Node(neighbor[0], neighbor[1], 0, 0, s.fScore, s.gScore, s.depth + 1)
                        nodes.append(node)

if __name__ == '__main__':
    main()
