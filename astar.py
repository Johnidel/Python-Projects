import heapq
import math
import time

#path finding algorithm

def SameSlope(path):
    pathNew = []
    pattern = 0,0
    prev = path[0]
    pathNew.append(path[0])
    for i in range(len(path)):
        if pattern[0]==0 and pattern[1]==0:
            if i+1 < len(path):
                pattern = (path[i+1][0] - path[i][0],path[i+1][1]-path[i][1])
                prev = path[i]
        else:
            if path[i][0]-prev[0]==pattern[0] and path[i][1]-prev[1]==pattern[1]:
                prev = path[i]
            else:
                pathNew.append(prev)
                pattern = (path[i][0]-prev[0],path[i][1]-prev[1])
                prev = path[i]
    pathNew.append(path[-1])
    return pathNew

class Node:
	def __init__(self, xPos, yPos, parent, distance, priority):
		self.x = xPos
		self.y = yPos
		self.parent = parent
		self.distance = distance
		self.priority = priority
	def updatePriority(self, dest):
		#update priority of node based on heuristic
		self.priority = round(self.distance + self.estimate(dest) * 5)
	def __lt__(self, other):
		return self.priority < other.priority
	def estimate(self, dest):
		#Euclidian
		return (((dest[0] - self.x) ** 2) + ((dest[1] - self. y) ** 2)) ** (1 / 2)
		#Manhattan
		#return (math.fabs(dest[0] - self.x) + math.fabs(dest[1] - self.y))
	def __repr__(self):
		return("Node" + str(self.x) + "  - " + str(self.y) + " P" + str(self.priority))

class AStar:
	def __init__(self, grid_, start, goal):
		self.grid = grid_
		self.goal = goal
		self.open_nodes = set()
		self.closed_nodes = set()
		self.queue = []
		self.openNodes = [[0 for i in range(len(self.grid[0]))] for j in range(len(self.grid))]
		self.closedNodes = [[0 for i in range(len(self.grid[0]))] for j in range(len(self.grid))]
		self.path = []
		self.notFound = True

	def startPath(self, start, goal):
		self.start = start
		self.goal = goal
		startNode = Node(start[0] - 1, start[1] - 1, None, 0, 0)
		startNode.updatePriority(goal)
		heapq.heappush(self.queue, startNode)
		if startNode.y > 32:
			startNode.y = 31
		elif startNode.y < 0:
			startNode.y = 1
		if startNode.x > 24:
			startNode.x = 23
		elif startNode.x < 0:
			startNode.x = 1
		self.openNodes[startNode.x][startNode.y] = 0
		self.closedNodes[startNode.x][startNode.y] = 1
		self.notFound = True
		self.grid[startNode.x][startNode.y] = 0
		self.grid[goal[0]][goal[1]] = 0

	def pathFind(self, TIME_PERMITTED):
		#send in the time we are allowing to compute this part 
		#of the path
		#(path not found at once, partitions path finding)
		dx = [1, -1, 0, 0, 1, 1, -1, -1]
		dy = [0, 0, 1, -1, 1, -1, 1, -1]
		timeStart = time.time()
		while len(self.queue) > 0 and self.notFound:

			if time.time() - timeStart > TIME_PERMITTED:
				pathList = []
				while curNode.parent:
					pathList.append((curNode.x, curNode.y))
					curNode = curNode.parent
				self.path = pathList[::-1]

			curNode = heapq.heappop(self.queue)
			x = curNode.x
			y = curNode.y
			self.openNodes[x][y] = 0
			self.closedNodes[x][y] = 1

			if x == self.goal[0] and y == self.goal[1]:
				pathList = []
				self.notFound = False
				while curNode.parent:
					pathList.append((curNode.x, curNode.y))
					curNode = curNode.parent
				self.path = pathList[::-1]
				return SameSlope(self.path)

			for i in range(len(dx)):
				childX = x + dx[i]
				childY = y + dy[i]

				if not childX < 0 and not childX > len(self.grid) - 1 and not childY < 0 and not childY > len(self.grid[0]) - 1 and self.grid[childX][childY] != 1 and self.closedNodes[childX][childY] != 1:
					if dy[i] != 0 and dy[i] != 0:
						if self.grid[childX][y] == 1 or self.grid[x][childY] == 1:
							continue
					if i < 4:
						childNode = Node(childX, childY, curNode, curNode.distance + 10, curNode.priority)
					else:
						childNode = Node(childX, childY, curNode, curNode.distance + 12, curNode.priority)
					childNode.updatePriority(self.goal)
					if self.openNodes[childX][childY] == 0:
						self.openNodes[childX][childY] = childNode.priority
						heapq.heappush(self.queue, childNode)
					elif self.openNodes[childX][childY] < childNode.priority:
						self.openNodes[childX][childY] = childNode.priority
						for i2 in range(len(self.queue)):
							if self.queue[i2].x == childX and self.queue[i2].y == childY:
								self.queue[i2] = childNode
								break
						heapq.heapify(self.queue)
		if len(self.queue) == 0:
			return None
