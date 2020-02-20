import math
import heapq
import sys
import os


class MyVertex:
    def __init__(self, x, y, type=0):
        self.x = x
        self.y = y
        self.f = 999999999
        self.g = 999999999
        self.track = None
        self.type = type

    """type: 0: possible step, 1: obstacle, 2: start, 3: goal"""

    def corPrint(self):
        return "(%d,%d)" % (self.y, self.x)

    def print(self):
        if self.type == 0:
            print("-", end=" ")
        elif self.type == 1:
            print("o", end=" ")
        elif self.type == 2:
            print("S", end=" ")
        elif self.type == 3:
            print("G", end=" ")
        else:
            print("x", end=" ")

    def setType(self, type):
        self.type = type

    def getCor(self):
        return (self.x, self.y)

    def getType(self):
        return self.type

    def __lt__(self, other):
        return self.f < other.f

    def __le__(self, other):
        return self.f <= other.f

    def __gt__(self, other):
        return self.f > other.f

    def __ge__(self, other):
        return self.f >= other.f

class Map:
    priority_step = ((-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0))

    def __init__(self):
        self.mapArr = []
        self.rows = 0
        self.cols = 0
        self.solve = 0
        self.path = []

    def euclidHeuristic(self, v1, v2):
        return math.sqrt(math.pow(v1.x - v2.x, 2) + math.pow(v1.y - v2.y, 2))

    def setMap(self, rows, cols, describeTable):
        #print(rows[0])
        self.rows = rows[0]
        self.cols = cols[0]
        self.mapArr = [None] * self.rows
        for i in range(self.rows):
            self.mapArr[i] = [None] * self.cols
        for i in range(len(describeTable)):
            for j in range(len(describeTable[i])):
                if (describeTable[i][j] == 0):
                    self.mapArr[i][j] = MyVertex(j, i, 0)
                elif (describeTable[i][j] == 1):
                    self.mapArr[i][j] = MyVertex(j, i, 1)
                elif (describeTable[i][j] == 2):
                    self.start = MyVertex(j, i, 2)
                    self.mapArr[i][j] = self.start
                else:
                    self.goal = MyVertex(j, i, 3)
                    self.mapArr[i][j] = self.goal

    def setMapFromFile(self, fileName):
        if os.path.isfile(fileName) == False:
            print("File does not exist or unreadable!")
            return 0

        f = open(fileName, "r")

        lines_list = f.readlines()
        rows = [int(val) for val in lines_list[0].split()]
        xStart, yStart = [int(val) for val in lines_list[1].split()]
        xGoal, yGoal = [int(val) for val in lines_list[2].split()]
        array = [[int(val) for val in line.split()] for line in lines_list[3:]]
        array[xStart][yStart] = 2
        array[yGoal][yGoal] = 3
        self.setMap(rows, rows, array)
        f.close()
        return 1

    def printMapToFile(self, fileName):
        f = open(fileName, "w")
        if self.solve == 0:
            f.write("-1")
        else:
            f.write(str(len(self.path)))
            f.write("\n")
            temp = ""
            for i in range(0,len(self.path)):
                temp += "(%d,%d) "%(self.path[i].y, self.path[i].x)
            f.write(temp)
            f.write("\n")
            for i in range(0, self.rows):
                temp = ""
                for j in range(0, self.cols):
                    if self.mapArr[i][j].type == 0:
                        temp += "- "
                    elif self.mapArr[i][j].type == 1:
                        temp += "o "
                    elif self.mapArr[i][j].type == 2:
                        temp += "S "
                    elif self.mapArr[i][j].type == 3:
                        temp += "G "
                    else:
                        temp += "x "
                temp += '\n'
                f.write(temp)
        f.close()

    def findPath_AStarAlgorithm(self, inputFileName, outputFileName):
        if self.setMapFromFile(inputFileName) == 0:
            return

        closeSet = []
        openSet = []
        openSet.append(self.start);
        heapq.heapify(openSet)
        self.start.g = 0
        current = self.start
        while (len(openSet) != 0):
            current = heapq.heappop(openSet)

            if (current == self.goal):
                break
            closeSet.append(current)
            for i in range(len(self.priority_step)):
                xNext = current.x + self.priority_step[i][0]
                yNext = current.y + self.priority_step[i][1]
                if (xNext < 0 or xNext >= self.cols or yNext < 0 or yNext >= self.rows or self.mapArr[yNext][
                    xNext].getType() == 1):
                    continue

                next = self.mapArr[yNext][xNext]
                if (next in closeSet):
                    continue
                diff = current.g + 1

                if (diff >= next.g):
                    continue
                next.g = diff
                next.f = diff + self.euclidHeuristic(next, self.goal)
                next.track = current
                if (next not in openSet):
                    heapq.heappush(openSet, next)

        self.path.clear()
        if (current == self.goal):
            self.path.append(current)
            current = current.track
            while current != self.start:
                current.type = 4
                self.path.append(current)
                current = current.track
            self.path.append(current)
            self.path.reverse()
            self.solve = 1

        self.printMapToFile(outputFileName)
        print("Completed!")

if __name__ == '__main__':
    if len(sys.argv) == 3:
        map = Map()
        map.findPath_AStarAlgorithm(sys.argv[1], sys.argv[2])
    else:
        print("Command is incorrect!")
        print("Format: executableFile inputFileName outputFileName")

