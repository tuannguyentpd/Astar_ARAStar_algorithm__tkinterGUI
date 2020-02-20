import math
import heapq
import sys
import os
import time as t

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

    def isVisited(self, List):
        for i in range(len(List)):
            if self.getCor() == List[i].getCor():
                return True
        return False

class Map:
    priority_step = ((-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0))

    def __init__(self):
        self.timeLimit = 10000
        self.mapArr = []
        self.rows = 0
        self.cols = 0
        self.solve = 0
        self.path = []

    def maxDxDyHeuristic(self, v1, v2):
        return max(abs(v1.x-v2.x),abs(v1.y-v2.y))

    def setMap(self, time, rows, cols, describeTable):
        self.timeLimit = time[0]
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
        time = [int(val) for val in lines_list[0].split()]
        rows = [int(val) for val in lines_list[1].split()]
        xStart, yStart = [int(val) for val in lines_list[2].split()]
        xGoal, yGoal = [int(val) for val in lines_list[3].split()]
        array = [[int(val) for val in line.split()] for line in lines_list[4:]]
        array[xStart][yStart] = 2
        array[yGoal][yGoal] = 3
        self.setMap(time, rows, rows, array)
        f.close()
        return 1

    def printMapToFile(self, f, time, e):
        f.write("time: %f milliseconds"%(time))
        f.write("\n")
        f.write("e: %f"%(e))
        f.write("\n")
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
        f.write("\n")

    def improvePath(self, inconsSet, openSet, closeSet, e):
        next = None
        while self.goal.f > openSet[0].f:
            smallest = heapq.heappop(openSet)
            if smallest not in closeSet:
                closeSet.append(smallest)

            for i in range(len(self.priority_step)):
                xNext = smallest.x + self.priority_step[i][0]
                yNext = smallest.y + self.priority_step[i][1]
                if (xNext < 0 or xNext >= self.cols or yNext < 0 or yNext >= self.rows or self.mapArr[yNext][
                    xNext].getType() == 1):
                    continue

                next = self.mapArr[yNext][xNext]

                if next.isVisited(closeSet) and next.isVisited(openSet) and next.isVisited(inconsSet):
                    next.g = math.inf
                if next.g > smallest.g + 1:
                    next.g = smallest.g + 1
                    next.f = next.g + e * self.maxDxDyHeuristic(next, self.goal)
                    next.track = smallest

                    if next not in closeSet:
                        heapq.heappush(openSet, next)
                    else:
                        inconsSet.append(next)

                    if next == self.goal:
                        closeSet.append(next)
                        break
            if len(openSet) == 0:
                break

        if len(closeSet) == 0 or self.goal.x != closeSet[len(closeSet) - 1].x or self.goal.y != closeSet[
            len(closeSet) - 1].y:
            return False
        else:
            return True

    def calE(self, openSet, inconsSet, e):
        if len(inconsSet) == 0 and len(openSet) == 0:
            return e;
        Min = math.inf
        for i in range(len(openSet)):
            tem = openSet[i].g + self.maxDxDyHeuristic(openSet[i], self.goal)
            if tem < Min: Min = tem
        for i in range(len(inconsSet)):
            tem = inconsSet[i].g + self.maxDxDyHeuristic(inconsSet[i], self.goal)
            if tem < Min: Min = tem
        Min = self.goal.g / Min
        if e < Min: return e
        return Min

    def calRunTime(self, startTime):
        return (t.time() - startTime) * 1e3

    def findPath_AraStarAlgorithm(self, inputFileName, outputFileName):
        if self.setMapFromFile(inputFileName) == 0:
            return

        g = open(outputFileName,"w")
        g.write("")
        g.close()

        f = open(outputFileName, "a")

        e = 2.5

        self.goal.g = self.goal.f = math.inf
        self.start.g = 0
        self.start.f = self.start.g + e * self.maxDxDyHeuristic(self.start, self.goal)
        closeSet = []
        openSet = []
        inconsSet = []

        heapq.heappush(openSet, self.start)

        startTime = t.time()
        solve = self.improvePath(inconsSet, openSet, closeSet, e)

        diffTime = self.calRunTime(startTime)
        if not solve or diffTime > self.timeLimit:
            f.write("-1")
            f.close()
            print("Completed!")
            return

        e1 = self.calE(openSet, inconsSet, e)
        self.traceSolution(self.goal)
        self.printMapToFile(f, diffTime, e)
        self.deleteSolution()
        while (e1 > 1):
            e = e - 0.5

            temp = openSet;
            openSet = [];
            for i in range(len(temp)):
                temp[i].f = temp[i].g + e * self.maxDxDyHeuristic(temp[i], self.goal)
                heapq.heappush(openSet, temp[i])
            while len(inconsSet) != 0:
                inconsSet[0].f = inconsSet[0].g + e * self.maxDxDyHeuristic(inconsSet[0], self.goal)
                heapq.heappush(openSet, inconsSet.pop(0))

            closeSet = []

            self.improvePath(inconsSet, openSet, closeSet, e)

            diffTime = self.calRunTime(startTime)
            if diffTime <= self.timeLimit:
                self.traceSolution(self.goal)
                self.printMapToFile(f, diffTime, e)
                self.deleteSolution()
            else:
                f.close()
                print("Completed!")
                return
            e1_new = self.calE(inconsSet, openSet, e)

            if e1_new < e1:
                e1 = e1_new
        f.close()
        print("Completed!")

    def traceSolution(self, s):
        self.path.clear()
        if s == self.goal:
            self.path.insert(0, s)
            s = s.track
            while s != self.start:
                s.type = 4
                self.path.insert(0, s)
                s = s.track
            self.path.insert(0, self.start)

    def deleteSolution(self):
        for i in range(0,len(self.path)):
            if self.path[i].type != 2 and self.path[i].type != 3:
                self.mapArr[self.path[i].y][self.path[i].x].type = 0

if __name__ == '__main__':
    if len(sys.argv) == 3:
        map = Map()
        map.findPath_AraStarAlgorithm(sys.argv[1], sys.argv[2])
    else:
        print("Command is incorrect!")
        print("Format: executableFile inputFileName outputFileName")

