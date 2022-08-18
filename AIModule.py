from ast import Pass
from copy import deepcopy
from queue import PriorityQueue
from telnetlib import DO
from tokenize import ContStr, Floatnumber
from Point import Point
import math

def minNumOfSteps(p1, p2):
	dx = abs(p2.x - p1.x)
	dy = abs(p2.y - p1.y)
	return min(dx, dy) + max(dy-dx, dx-dy)
	
'''AIModule Interface 
createPath(map map_) -> list<points>: Adds points to a path'''
class AIModule:
	def createPath(self, map_):
		pass

'''
A sample AI that takes a very sub optimal path.
This is a sample AI that moves as far horizontally as necessary to reach
the target, then as far vertically as necessary to reach the target.
It is intended primarily as a demonstration of the various pieces of the
program.
'''

class StupidAI(AIModule):

	def createPath(self, map_):
		path = []
		explored = []
		# Get starting point
		path.append(map_.start)
		current_point = deepcopy(map_.start)

		# Keep moving horizontally until we match the target
		while(current_point.x != map_.goal.x):
			# If we are left of goal, move right
			if current_point.x < map_.goal.x:
				current_point.x += 1
			# If we are right of goal, move left
			else:
				current_point.x -= 1
			path.append(deepcopy(current_point))

		# Keep moving vertically until we match the target
		while(current_point.y != map_.goal.y):
			# If we are left of goal, move right
			if current_point.y < map_.goal.y:
				current_point.y += 1
			# If we are right of goal, move left
			else:
				current_point.y -= 1
			path.append(deepcopy(current_point))

		# We're done!
		return path

class Djikstras(AIModule):

	def createPath(self, map_):
		q = PriorityQueue()
		cost = {}
		prev = {}
		explored = {}
		for i in range(map_.width):
			for j in range(map_.length):
				cost[str(i)+','+str(j)] = math.inf
				prev[str(i)+','+str(j)] = None
				explored[str(i)+','+str(j)] = False
		current_point = deepcopy(map_.start)
		current_point.comparator = 0
		cost[str(current_point.x)+','+str(current_point.y)] = 0
		q.put(current_point)
		while q.qsize() > 0:
			# Get new point from PQ
			v = q.get()
			if explored[str(v.x)+','+str(v.y)]:
				continue
			explored[str(v.x)+','+str(v.y)] = True
			# Check if popping off goal
			if v.x == map_.getEndPoint().x and v.y == map_.getEndPoint().y:
				break
			# Evaluate neighbors
			neighbors = map_.getNeighbors(v)
			for neighbor in neighbors:
				''##############################################''
				alt = map_.getCost(v, neighbor) + cost[str(v.x)+','+str(v.y)]
				if alt < cost[str(neighbor.x)+','+str(neighbor.y)]:
					cost[str(neighbor.x)+','+str(neighbor.y)] = alt
					neighbor.comparator = alt
					prev[str(neighbor.x)+','+str(neighbor.y)] = v
				q.put(neighbor)

		path = []
		while not(v.x == map_.getStartPoint().x and v.y == map_.getStartPoint().y):
			path.append(v)
			v = prev[str(v.x)+','+str(v.y)]
		path.append(map_.getStartPoint())
		path.reverse()
		return path

class AStarExp(AIModule):	
	def createPath(self, map_):
		def getHeuristic(p1, p2):
			if p1.x == p2.x and p1.y == p2.y:
				return 0
			minStep = minNumOfSteps(p1, p2)
			heightDiff = map_.getTile(p2.x, p2.y) - map_.getTile(p1.x, p1.y)
			uod = math.floor(heightDiff/minStep)
			return minStep * math.pow(2,uod)

		q = PriorityQueue()
		cost = {}
		prev = {}
		explored = {}
		endPoint = deepcopy(map_.getEndPoint())
		for i in range(map_.width):
			for j in range(map_.length):
				cost[str(i)+','+str(j)] = math.inf
				prev[str(i)+','+str(j)] = None
				explored[str(i)+','+str(j)] = False
		current_point = deepcopy(map_.start)
		current_point.comparator = getHeuristic(current_point,endPoint)
		cost[str(current_point.x)+','+str(current_point.y)] = 0
		q.put(current_point)
		while q.qsize() > 0:
			# Get new point from PQ
			v = q.get()
			if explored[str(v.x)+','+str(v.y)]:
				continue
			explored[str(v.x)+','+str(v.y)] = True
			# Check if popping off goal
			if v.x == endPoint.x and v.y == endPoint.y:
				break
			# Evaluate neighbors
			neighbors = map_.getNeighbors(v)
			for neighbor in neighbors:	
				alt = map_.getCost(v, neighbor) + cost[str(v.x)+','+str(v.y)]
				if alt < cost[str(neighbor.x)+','+str(neighbor.y)]:
					cost[str(neighbor.x)+','+str(neighbor.y)] = alt
					neighbor.comparator = getHeuristic(neighbor, endPoint) + alt
					prev[str(neighbor.x)+','+str(neighbor.y)] = v
				if (current_point.comparator <= neighbor.comparator):
					q.put(neighbor)

		path = []		
		while not(v.x == map_.getStartPoint().x and v.y == map_.getStartPoint().y):
			path.append(v)
			v = prev[str(v.x)+','+str(v.y)]
		path.append(map_.getStartPoint())
		path.reverse()
		return path

class AStarDiv(AIModule):
	def createPath(self, map_):
		def getHeuristic(p1, p2):
			if p1.x == p2.x and p1.y == p2.y:
				return 0
			minStep = minNumOfSteps(p1, p2)
			heightDiff = map_.getTile(p2.x, p2.y) - map_.getTile(p1.x, p1.y)
			p = p1
			if (map_.getTile(p1.x, p1.y) > map_.getTile(p2.x, p2.y)):
				p = p2
			return (minStep - 1) * map_.getCost(p, p)
		
		q = PriorityQueue()
		cost = {}
		prev = {}
		explored = {}
		endPoint = deepcopy(map_.getEndPoint())
		for i in range(map_.width):
			for j in range(map_.length):
				cost[str(i)+','+str(j)] = math.inf
				prev[str(i)+','+str(j)] = None
				explored[str(i)+','+str(j)] = False
		current_point = deepcopy(map_.start)
		current_point.comparator = getHeuristic(current_point,endPoint)
		cost[str(current_point.x)+','+str(current_point.y)] = 0
		q.put(current_point)
		while q.qsize() > 0:
			# Get new point from PQ
			v = q.get()
			if explored[str(v.x)+','+str(v.y)]:
				continue
			explored[str(v.x)+','+str(v.y)] = True
			# Check if popping off goal
			if v.x == endPoint.x and v.y == endPoint.y:
				break
			# Evaluate neighbors
			neighbors = map_.getNeighbors(v)
			for neighbor in neighbors:	
				alt = map_.getCost(v, neighbor) + cost[str(v.x)+','+str(v.y)]
				if alt < cost[str(neighbor.x)+','+str(neighbor.y)]:
					cost[str(neighbor.x)+','+str(neighbor.y)] = alt
					neighbor.comparator = getHeuristic(neighbor, endPoint) + alt
					prev[str(neighbor.x)+','+str(neighbor.y)] = v
				if (current_point.comparator <= neighbor.comparator):
					q.put(neighbor)

		path = []		
		while not(v.x == map_.getStartPoint().x and v.y == map_.getStartPoint().y):
			path.append(v)
			v = prev[str(v.x)+','+str(v.y)]
		path.append(map_.getStartPoint())
		path.reverse()
		return path

class AStarMSH(AIModule):
	def createPath(self, map_):
		def getHeuristic(p1, p2):
			if p1.x == p2.x and p1.y == p2.y:
				return 0
			minStep = minNumOfSteps(p1, p2)
			heightDiff = map_.getTile(p2.x, p2.y) - map_.getTile(p1.x, p1.y)
			uod = 0
			if heightDiff < 0:
				uod = -1
			if heightDiff > 0:
				uod = 1
			if minStep >= heightDiff:  
				return minStep + abs(heightDiff) * (math.pow(2,uod)-1)
			uod = math.floor(heightDiff/minStep)
			return minStep * math.pow(2,uod)
		
		q = PriorityQueue()
		cost = {}
		prev = {}
		explored = {}
		for i in range(map_.width):
			for j in range(map_.length):
				cost[str(i)+','+str(j)] = math.inf
				prev[str(i)+','+str(j)] = None
				explored[str(i)+','+str(j)] = False
		endPoint = deepcopy(map_.getEndPoint())
		current_point = deepcopy(map_.start)
		current_point.comparator = getHeuristic(current_point,endPoint)
		cost[str(current_point.x)+','+str(current_point.y)] = 0
		q.put(current_point)
		while q.qsize() > 0:
			# Get new point from PQ
			v = q.get()
			if explored[str(v.x)+','+str(v.y)]:
				continue
			explored[str(v.x)+','+str(v.y)] = True
			# Check if popping off goal
			if v.x == endPoint.x and v.y == endPoint.y:
				break
			# Evaluate neighbors
			neighbors = map_.getNeighbors(v)
			for neighbor in neighbors:
				alt = map_.getCost(v, neighbor) + cost[str(v.x)+','+str(v.y)]
				if alt < cost[str(neighbor.x)+','+str(neighbor.y)]:
					cost[str(neighbor.x)+','+str(neighbor.y)] = alt
					neighbor.comparator = getHeuristic(neighbor, endPoint) + alt
					prev[str(neighbor.x)+','+str(neighbor.y)] = v
				if (current_point.comparator <= neighbor.comparator):
					q.put(neighbor)
		
		path = []		
		while not(v.x == map_.getStartPoint().x and v.y == map_.getStartPoint().y):
			path.append(v)
			v = prev[str(v.x)+','+str(v.y)]
		path.append(map_.getStartPoint())
		path.reverse()
		return path
			