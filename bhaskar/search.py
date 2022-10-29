# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    coordinates = "coordinates"
    path = "path"
    stack = util.Stack()
    visited = set()
    start = {coordinates : problem.getStartState(), path : []}
    stack.push(start)
    while not stack.isEmpty():
        curr = stack.pop()
        visited.add(curr[coordinates])
        # print(curr)
        if problem.isGoalState(curr[coordinates]):
            # print(curr[path])
            return curr[path]
        successors = problem.getSuccessors(curr[coordinates])
        for child in successors:
            if child[0] in visited:
                continue
            stack.push({coordinates : child[0], path : curr[path]+[child[1]]})
    return []
    # util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    coordinates = "coordinates"
    path = "path"
    nodes = "nodes"
    queue = util.Queue()
    visited = set()
    start = {coordinates : problem.getStartState(), path : [], nodes :[problem.getStartState()]}
    queue.push(start)
    visited.add(start[coordinates])
    while not queue.isEmpty():
        curr = queue.pop()
        if problem.isGoalState(curr[coordinates]):
            # print("ans ", curr)
            return curr[path]
        # print(curr)
        successors = problem.getSuccessors(curr[coordinates])
        for child in successors:
            if child[0] in visited:
                continue
            visited.add(child[0])
            nextPath = curr[path]+[child[1]]
            newNodes = curr[nodes] + [child[0]]
            queue.push({coordinates : child[0], path : nextPath, nodes: newNodes})
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    PQ = util.PriorityQueue()
    coordinates = "coordinates"
    path = "path"
    pathCost = "pathCost"
    visited = set()
    start = problem.getStartState()
    visited.add(start)
    PQ.push(start, 0)
    pathMap = {}
    frontier = set()
    frontier.add(start)
    pathMap[start]= {path:[], pathCost: 0}
    while not PQ.isEmpty():
        curr = PQ.pop()
        if curr in frontier:
            frontier.remove(curr)
        if problem.isGoalState(curr):
            # print(pathMap[curr][path])
            return pathMap[curr][path]
        # print(curr)
        visited.add(curr)
        successors = problem.getSuccessors(curr)
        for child in successors:
            nextPath = pathMap[curr][path]+[child[1]]
            # print(child)
            nextCost = pathMap[curr][pathCost] + child[2]
            if child[0] in frontier:
                if nextCost < pathMap[child[0]][pathCost]:
                    PQ.update(child[0], nextCost)
                    pathMap[child[0]][path] = nextPath
                    pathMap[child[0]][pathCost] = nextCost
                continue
            if child[0] in visited:
                continue
            pathMap[child[0]] = {path: nextPath, pathCost: nextCost}
            frontier.add(child[0])
            PQ.push(child[0], nextCost)
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    # print(state)
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    # temp = heuristic
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    PQ = util.PriorityQueue()
    coordinates = "coordinates"
    path = "path"
    pathCost = "pathCost"
    visited = set()
    start = problem.getStartState()
    visited.add(start)
    PQ.push(start, 0)
    pathMap = {}
    frontier = set()
    frontier.add(start)
    visited.add(start)
    pathMap[start]= {path:[], pathCost: 0}
    while not PQ.isEmpty():
        curr = PQ.pop()
        if curr in frontier:
            frontier.remove(curr)
        if problem.isGoalState(curr):
            # print(pathMap[curr][path])
            return pathMap[curr][path]
        # print(curr)
        successors = problem.getSuccessors(curr)
        for child in successors:
            nextPath = pathMap[curr][path]+[child[1]]
            # print("check", pathMap[curr][pathCost])
            nextCost = pathMap[curr][pathCost] + child[2] + heuristic(child[0], problem)
            nextCostWithoutHeuristic = pathMap[curr][pathCost] + child[2]
            if child[0] in frontier:
                if nextCost < pathMap[child[0]][pathCost]:
                    PQ.update(child[0], nextCost)
                    pathMap[child[0]][path] = nextPath
                    pathMap[child[0]][pathCost] = nextCostWithoutHeuristic
                continue
            if child[0] in visited:
                continue
            visited.add(child[0])
            pathMap[child[0]] = {path: nextPath, pathCost: nextCostWithoutHeuristic}
            frontier.add(child[0])
            PQ.push(child[0], nextCost)
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
