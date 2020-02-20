import queue
import math
import heapq
import tkinter
from tkinter import filedialog
import time
from tkinter import messagebox
import queue
from random import randrange
from tkinter import ttk

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
        self.mapArr = []
        self.rows = 0
        self.cols = 0
        self.path = []
        self.heuristic=self.euclidHeuristic

    def euclidHeuristic(self, v1, v2):
        return math.sqrt(math.pow(v1.x - v2.x, 2) + math.pow(v1.y - v2.y, 2))

    def maxDxDyHeuristic(self, v1, v2):
        return max(abs(v1.x-v2.x),abs(v1.y-v2.y))

    def setMap(self, rows, cols, describeTable):
        self.rows = rows
        self.cols = cols
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

    def findPath_AStarAlgorithm(self, GUI_object=None):
        openSet = []
        openSet.append(self.start);
        heapq.heapify(openSet)
        self.start.g = 0
        current = self.start
        while (len(openSet) != 0):
            current = heapq.heappop(openSet)

            if (current == self.goal):
                break
            if (current is not self.start):
                GUI_object.drawMap(current.x, current.y, 6)
            for i in range(len(self.priority_step)):
                xNext = current.x + self.priority_step[i][0]
                yNext = current.y + self.priority_step[i][1]
                if (xNext < 0 or xNext >= self.cols or yNext < 0 or yNext >= self.rows or self.mapArr[yNext][
                    xNext].getType() == 1):
                    continue

                next = self.mapArr[yNext][xNext]

                diff = current.g + 1

                if (diff >= next.g):
                    continue
                next.g = diff
                next.f = diff + self.heuristic(next, self.goal)
                next.track = current
                if (next not in openSet):
                    heapq.heappush(openSet, next)
                    if (next is not self.goal):
                        GUI_object.drawMap(next.x, next.y, 5)

        # print("Done\n")
        self.path.clear()
        if (current == self.goal):
            self.path.insert(0, current)
            current = current.track
            while current != self.start:
                current.type = 4
                GUI_object.drawMap(current.x, current.y, current.type)
                self.path.insert(0, current)
                current = current.track
            self.path.insert(0, self.start)
            GUI_object.showStepNumber(len(self.path))
            return 1
        else:
            return 0

    ############################### ARA STAR ##################################
    def improvePath(self, inconsSet, openSet, closeSet, e, GUI_object=None):
        next = None
        while self.goal.f > openSet[0].f:
            smallest = heapq.heappop(openSet)
            if smallest not in closeSet:
                closeSet.append(smallest)
            if (smallest is not self.start):
                if smallest.type != 4:
                    GUI_object.drawMap(smallest.x, smallest.y, 6)
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
                    next.f = next.g + e * self.heuristic(next, self.goal)
                    next.track = smallest

                    if next not in closeSet:
                        heapq.heappush(openSet, next)
                    else:
                        inconsSet.append(next)

                    if next is not self.goal:
                        if next is not self.start:
                            if next.type != 4:
                                GUI_object.drawMap(next.x, next.y, 5)
                    else:
                        closeSet.append(next)
                        break
            if len(openSet) == 0:
                break;

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
            tem = openSet[i].g + self.heuristic(openSet[i], self.goal)
            if tem < Min: Min = tem
        for i in range(len(inconsSet)):
            tem = inconsSet[i].g + self.heuristic(inconsSet[i], self.goal)
            if tem < Min: Min = tem
        Min = self.goal.g / Min
        if e < Min: return e
        return Min

    def findPath_AraStarAlgorithm(self, GUI_object=None):
        e = 2.5

        self.goal.g = self.goal.f = math.inf
        self.start.g = 0
        self.start.f = self.start.g + e * self.heuristic(self.start, self.goal)
        closeSet = []
        openSet = []
        inconsSet = []

        heapq.heappush(openSet, self.start)
        solve = self.improvePath(inconsSet, openSet, closeSet, e, GUI_object)
        if not solve:
            return 0

        e1 = self.calE(openSet, inconsSet, e)
        self.printSolution(self.goal, GUI_object)
        while (e1 > 1):
            e = e - 0.5

            temp = openSet;
            openSet = [];
            for i in range(len(temp)):
                temp[i].f = temp[i].g + e * self.heuristic(temp[i], self.goal)
                heapq.heappush(openSet, temp[i])
            while len(inconsSet) != 0:
                inconsSet[0].f = inconsSet[0].g + e * self.heuristic(inconsSet[0], self.goal)
                heapq.heappush(openSet, inconsSet.pop(0))

            closeSet = []

            self.improvePath(inconsSet, openSet, closeSet, e, GUI_object)
			
            e1_new = self.calE(inconsSet, openSet, e)
            if e1_new < e1:
                e1 = e1_new

            self.printSolution(self.goal, GUI_object)

        self.printSolutionFinal(self.goal, GUI_object)
        return 1

    def printSolution(self, s, GUI_object=None):
        self.path.clear()
        if s == self.goal:
            self.path.insert(0, s)
            s = s.track
            while s != self.start:
                s.type = 4
                GUI_object.drawMap(s.x, s.y, s.type)
                self.path.insert(0, s)
                s = s.track
            self.path.insert(0, self.start)
            GUI_object.showStepNumber(len(self.path))
            return 1
        else:
            return 0

    def printSolutionFinal(self, s, GUI_object=None):
        self.path.clear()
        if s == self.goal:
            self.path.insert(0, s)
            s = s.track
            while s != self.start:
                s.type = 4
                GUI_object.drawMap(s.x, s.y, 7)
                self.path.insert(0, s)
                s = s.track
            self.path.insert(0, self.start)
            GUI_object.showStepNumber(len(self.path))
            return 1
        else:
            return 0

    #################### ARA STAR ################################

    def print(self):
        for i in range(len(self.mapArr)):
            for j in range(len(self.mapArr[i])):
                self.mapArr[i][j].corPrint()

    # def findPath(self):

    def printMap(self):
        for i in range(len(self.mapArr)):
            for j in range(len(self.mapArr[i])):
                self.mapArr[i][j].print();
            print("\n")

    def printFile(self, fileName):
        f = open(fileName, "w")
        if (len(self.path) == 0):
            f.write("-1")
            f.close()
            return
        f.write("%d\n" % len(self.path))
        for i in range(len(self.path)):
            f.write(self.path[i].corPrint())
        f.write("\n")

        for i in range(len(self.mapArr)):
            for j in range(len(self.mapArr[0])):
                if (self.mapArr[i][j].type == 0):
                    f.write("- ")
                elif (self.mapArr[i][j].type == 1):
                    f.write("o ")
                elif (self.mapArr[i][j].type == 2):
                    f.write("S ")
                elif (self.mapArr[i][j].type == 3):
                    f.write("G ")
                else:
                    f.write("x ")
            f.write("\n")
        f.close()


class GUI_Map:
    backgoundColor = "#%02x%02x%02x" % (255, 255, 255)
    colorType = {'possible step': "#%02x%02x%02x" % (255, 255, 255), 'obstacle': "#%02x%02x%02x" % (100, 100, 100),
                 'start': "#%02x%02x%02x" % (100, 100, 255), 'goal': "#%02x%02x%02x" % (255, 100, 100),
                 'path_final': "#%02x%02x%02x" % (100, 255, 100), 'openStep': "#%02x%02x%02x" % (200, 200, 200),
                 'closeStep': "#%02x%02x%02x" % (218, 65, 235), 'path': "#%02x%02x%02x" % (0, 245, 255)}
    cellSize = 12

    def __init__(self):
        self.fileName = None
        self.goal = None
        self.start = None
        self.mapTable = [[]]
        self.speed = 10
        self.map = None
        self.isHoldClick = False
        self.currentType = self.colorType['obstacle']
        self.mainWindow = tkinter.Tk()
        self.mainWindow.resizable(False, False)
        self.mainWindow.title("Illustrator")
        self.mainWindow.configure(background=self.backgoundColor)
        self.IllusPanel = tkinter.Canvas(master=self.mainWindow, width=800, height=500, bd=1, relief="flat",
                                         bg=self.backgoundColor)

        self.optionFrame = tkinter.Frame(master=self.mainWindow, width=250, height=350,
                                         bg="#%02x%02x%02x" % (255, 255, 255), borderwidth=2, relief="groove",
                                         highlightbackground="#%02x%02x%02x" % (180, 180, 180))

        self.StartButton = tkinter.Button(master=self.optionFrame, text="Start", bg=self.backgoundColor,
                                          relief="groove", width=10, command=lambda: self.solve())
        self.StartButton.place(x=140, y=270)
        self.NewButton = tkinter.Button(master=self.optionFrame, text="New", bg=self.backgoundColor, relief="groove",
                                        width=10, command=lambda: self.newButton_command())
        self.NewButton.place(x=20, y=270)
        self.rowsInputLabel = tkinter.Label(master=self.optionFrame, text="Number row", background=self.backgoundColor)
        self.rowsInputLabel.place(x=17, y=10)
        self.rowsInputEntry = tkinter.Entry(master=self.optionFrame, width=10)
        self.rowsInputEntry.place(x=20, y=30)
        self.rowsInputEntry.insert(0, "30")
        self.colsInputLabel = tkinter.Label(master=self.optionFrame, text="Number column",
                                            background=self.backgoundColor)
        self.colsInputLabel.place(x=137, y=10)
        self.colsInputEntry = tkinter.Entry(master=self.optionFrame, width=10)
        self.colsInputEntry.place(x=140, y=30)
        self.colsInputEntry.insert(0, "30")
        self.fileButton = tkinter.Button(master=self.optionFrame, text="Load File", bg=self.backgoundColor,
                                         relief="groove", width=10,
                                         command=lambda: self.fileDialog())
        self.fileButton.place(x=20, y=240)
        self.randomButton = tkinter.Button(master=self.optionFrame, text="Random Map", bg=self.backgoundColor,
                                           relief="groove", width=10,
                                           command=lambda: self.mazeGenerating())
        self.randomButton.place(x=140, y=240)
        self.algoChoiceCombobox = ttk.Combobox(master=self.optionFrame, values=["A* Algorithm", "ARA* Algorithm"],
                                               width=12)
        self.algoChoiceCombobox.place(x=20, y=140)
        self.algoChoiceLabel = tkinter.Label(master=self.optionFrame, text="Algorithm",
                                             background=self.backgoundColor)
        self.algoChoiceLabel.place(x=17, y=119)
        self.heuristicChoiceCombobox = ttk.Combobox(master=self.optionFrame, values=["Euclidean", "Max(dx,dy)"],
                                               width=12)
        self.heuristicChoiceCombobox.place(x=140, y=140)
        self.heuristicChoiceLabel = tkinter.Label(master=self.optionFrame, text="Heuristic",
                                             background=self.backgoundColor)
        self.heuristicChoiceLabel.place(x=137, y=119)
        self.algoChoiceCombobox.current(0)
        self.heuristicChoiceCombobox.current(0)

        self.speedScale = tkinter.Scale(self.optionFrame, from_=1, to=50, orient="horizontal", bg=self.backgoundColor,
                                        relief="flat", sliderlength=10, sliderrelief="flat", length=195)
        self.speedScale.place(x=20, y=190)
        self.speedScale.set(40)
        self.speedLabel = tkinter.Label(master=self.optionFrame, text="Speed",
                                        background=self.backgoundColor)
        self.speedLabel.place(x=17, y=170)
        self.placeStartPositionButton = tkinter.Button(master=self.optionFrame, text="Place Start",
                                                       bg=self.backgoundColor,
                                                       relief="groove", width=10,
                                                       command=lambda: self.placeStartPositionButton_commmand())
        self.placeStartPositionButton.place(x=20, y=60)
        self.placeGoalPositionButton = tkinter.Button(master=self.optionFrame, text="Place Goal",
                                                      bg=self.backgoundColor, relief="groove",
                                                      width=10,
                                                      command=lambda: self.placeGoalPositionButton_command())
        self.placeGoalPositionButton.place(x=140, y=60)
        self.showStepNumberLabel = tkinter.Label(master=self.optionFrame, text="", background=self.backgoundColor,
                                                 relief="groove", width=28)
        self.showStepNumberLabel.place(x=20, y=310)

        self.IllusPanel.bind("<ButtonPress-1>", self.clickOnIllustrator)
        self.IllusPanel.bind("<B1-Motion>", self.draggingOnIllustrator)

    def newButton_command(self):
        rows = int(self.rowsInputEntry.get())
        cols = int(self.colsInputEntry.get())
        width = cols * self.cellSize
        height = rows * self.cellSize
        if (width < 250):
            width = 250
        if (height < 350):
            height = 350
        self.mainWindow.geometry("%dx%d" % (width + 280, height + 20))
        self.IllusPanel.config(width=cols * self.cellSize + 1, height=rows * self.cellSize + 1)
        self.mapTable = [None] * rows
        for i in range(rows):
            self.mapTable[i] = [None] * cols
        for i in range(rows):
            for j in range(cols):
                self.mapTable[i][j] = self.IllusPanel.create_rectangle(j * self.cellSize + 3, i * self.cellSize + 3,
                                                                       (j + 1) * self.cellSize + 3,
                                                                       (i + 1) * self.cellSize + 3,
                                                                       outline='#%02x%02x%02x' % (180, 180, 180),
                                                                       fill='#%02x%02x%02x' % (255, 255, 255))
        self.IllusPanel.place(x=10, y=10)
        self.optionFrame.place(x=width + 20, y=10)
        self.fileButton["text"] = "Load File"
        self.IllusPanel.update()
        for child in self.optionFrame.winfo_children():
            child.configure(state='normal')

    def placeStartPositionButton_commmand(self):
        self.currentType = self.colorType['start']

    def placeGoalPositionButton_command(self):
        self.currentType = self.colorType['goal']

    def clickOnIllustrator(self, event):
        item = event.widget
        color = item.itemcget(tkinter.CURRENT, "fill")
        if (self.currentType == self.colorType['start']):
            if (self.start != None):
                self.IllusPanel.itemconfig(self.start, fill=self.colorType['possible step'])
            item.itemconfig(tkinter.CURRENT, fill=self.colorType['start'])
            i = (event.y - 3) // self.cellSize
            j = (event.x - 3) // self.cellSize
            self.start = self.mapTable[i][j]
            self.currentType = self.colorType['obstacle']

        elif (self.currentType == self.colorType['goal']):
            if (self.goal != None):
                self.IllusPanel.itemconfig(self.goal, fill=self.colorType['possible step'])
            item.itemconfig(tkinter.CURRENT, fill=self.colorType['goal'])
            i = (event.y - 3) // self.cellSize
            j = (event.x - 3) // self.cellSize
            self.goal = self.mapTable[i][j]
            self.currentType = self.colorType['obstacle']

        elif (color == self.colorType['goal'] or color == self.colorType['start']):
            return
        else:
            if (color == self.colorType['obstacle']):
                item.itemconfigure(tkinter.CURRENT, fill=self.colorType['possible step'])
            else:
                item.itemconfigure(tkinter.CURRENT, fill=self.colorType['obstacle'])

    def draggingOnIllustrator(self, event):
        item = self.IllusPanel.find_closest(event.x, event.y)
        if item:
            item_id = item[0]
            color = self.IllusPanel.itemcget(item_id, "fill")
            if (color == self.colorType['possible step']):
                self.IllusPanel.itemconfigure(item_id, fill=self.colorType['obstacle'])

    def fileDialog(self):
        if (self.fileButton.cget("text") == "Load File"):
            self.fileName = filedialog.askopenfilename(filetypes=(("Text File", "*.txt"), ("All Files", "*.*")),
                                                       title="Choose a file.")
            if (self.fileName == ""):
                return
            f = open(self.fileName, "r")
            lines_list = f.readlines()
            rows = [int(val) for val in lines_list[0].split()]
            xStart, yStart = [int(val) for val in lines_list[1].split()]
            xGoal, yGoal = [int(val) for val in lines_list[2].split()]
            array = [[int(val) for val in line.split()] for line in lines_list[3:]]
            array[xStart][yStart] = 2
            array[yGoal][yGoal] = 3
            self.rowsInputEntry.delete(0, "end")
            self.rowsInputEntry.insert(0, str(rows).strip('[]'))
            self.colsInputEntry.delete(0, "end")
            self.colsInputEntry.insert(0, str(rows).strip('[]'))
            self.newButton_command()
            for i in range(len(array)):
                for j in range(len(array[i])):
                    if (array[i][j] == 0):
                        self.IllusPanel.itemconfigure(self.mapTable[i][j], fill=self.colorType['possible step'])
                    elif (array[i][j] == 1):
                        self.IllusPanel.itemconfigure(self.mapTable[i][j], fill=self.colorType['obstacle'])
                    elif (array[i][j] == 2):
                        self.IllusPanel.itemconfigure(self.mapTable[i][j], fill=self.colorType['start'])
                        self.start = self.mapTable[i][j]
                    else:
                        self.IllusPanel.itemconfigure(self.mapTable[i][j], fill=self.colorType['goal'])
                        self.goal = self.mapTable[i][j]
        else:
            self.fileName = filedialog.asksaveasfilename(title="Choose a file.",
                                                         filetypes=(("Text File", "*.txt"), ("All Files", "*.*")),
                                                         confirmoverwrite=False)
            if (self.fileName == ""):
                return
            self.map.printFile(self.fileName)

    def drawMap(self, x, y, type):
        time.sleep(self.speed)

        if type == 0:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['possible step'])
        elif type == 1:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['obstacle'])
        elif type == 2:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['start'])
        elif type == 3:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['goal'])
        elif type == 4:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['path'])
        elif type == 5:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['openStep'])
        elif type == 6:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['closeStep'])
        else:
            self.IllusPanel.itemconfigure(self.mapTable[y][x], fill=self.colorType['path_final'])
        self.IllusPanel.update()

    def showStepNumber(self, number):
        self.showStepNumberLabel['text'] = str(number)+" steps"

    def solve(self):

        if (self.start == None):
            messagebox.showinfo("Error", "No found start point")
            return
        if (self.goal == None):
            messagebox.showinfo("Error", "No found goal point")
            return

        temp = [0] * len(self.mapTable)
        for i in range(len(self.mapTable)):
            temp[i] = [0] * len(self.mapTable[i])
        for i in range(len(self.mapTable)):
            for j in range(len(self.mapTable[i])):
                color = self.IllusPanel.itemcget(self.mapTable[i][j], "fill")
                if (color == self.colorType['possible step']):
                    continue
                if (color == self.colorType['obstacle']):
                    temp[i][j] = 1
                elif (color == self.colorType['start']):
                    temp[i][j] = 2
                elif (color == self.colorType['goal']):
                    temp[i][j] = 3
                else:
                    self.IllusPanel.itemconfigure(self.mapTable[i][j], fill=self.colorType['possible step'])
        self.speed = 51 - self.speedScale.get()
        self.speed /= 1000
        self.showStepNumberLabel['text']=""

        for child in self.optionFrame.winfo_children():
            child.configure(state='disable')

        self.map = Map()
        self.map.setMap(len(temp), len(temp[0]), temp)
        if(self.heuristicChoiceCombobox.get()=="Euclidean"):
            self.map.heuristic=self.map.euclidHeuristic
        else:
            self.map.heuristic = self.map.maxDxDyHeuristic
        if(self.algoChoiceCombobox.get()=="A* Algorithm"):
            if (self.map.findPath_AStarAlgorithm(self) == 0):
                messagebox.showinfo("Error", "No path found")
        else:
            if(self.map.findPath_AraStarAlgorithm(self) == 0):
                messagebox.showinfo("Error", "No path found")
        self.fileButton["text"] = "Save to File"
        for child in self.optionFrame.winfo_children():
            child.configure(state='normal')

    def mazeGenerating(self):
        self.newButton_command()
        rows = int(self.rowsInputEntry.get())
        cols = int(self.colsInputEntry.get())
        mazeArr = [1] * rows
        step = ((-2, 0), (0, 2), (2, 0), (0, -2))
        for i in range(rows):
            mazeArr[i] = [1] * cols
        candidatePoint = []
        stack = queue.LifoQueue()

        currentCell = (1, 1)
        iNext = 0
        jNext = 0
        stack.put(currentCell)
        mazeArr[currentCell[0]][currentCell[1]] = 0
        while (stack.empty() == False):
            candidatePoint.clear()
            for i in range(4):
                iNext = currentCell[0] + step[i][0]
                jNext = currentCell[1] + step[i][1]
                if (iNext < 0 or iNext >= rows or jNext < 0 or jNext >= cols or
                        mazeArr[iNext][jNext] == 0):
                    continue
                candidatePoint.append((iNext, jNext))
            if (len(candidatePoint) > 0):
                neighbour = candidatePoint[randrange(0, len(candidatePoint))]
                stack.put(currentCell)
                mazeArr[currentCell[0] + (neighbour[0] - currentCell[0]) // 2][
                    currentCell[1] + (neighbour[1] - currentCell[1]) // 2] = 0
                mazeArr[neighbour[0]][neighbour[1]] = 0
                currentCell = neighbour
            else:
                currentCell = stack.get()
        for i in range(rows):
            for j in range(cols):
                if (mazeArr[i][j] == 0):
                    self.IllusPanel.itemconfigure(self.mapTable[i][j], fill=self.colorType['possible step'])
                elif (mazeArr[i][j] == 1):
                    self.IllusPanel.itemconfigure(self.mapTable[i][j], fill=self.colorType['obstacle'])

    def run(self):
        self.newButton_command()
        self.mainWindow.mainloop()


if __name__ == '__main__':
    m = GUI_Map()
    m.run()
