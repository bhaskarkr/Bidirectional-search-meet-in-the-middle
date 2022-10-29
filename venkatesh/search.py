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
    return [s, s, w, s, w, w, s, w]


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

    # **Venkatesh Code Start Here**
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    # Initialization
    # import numpy as np
    from util import Stack
    fringe = Stack()  # The open-set
    fringe.push(problem.getStartState())
    explored = []
    pathStack = Stack()  # Stores the ways the pacman can go from a node with multiple successors
    completedPath = []

    while not fringe.isEmpty():
        node = fringe.pop()
        # print("The node popped out is: ", node)

        # Goal-test
        if problem.isGoalState(node):
            # print("!GOAL REACHED!")
            return completedPath

        if explored.count(node) == 0:
            explored.append(node)
            for x in problem.getSuccessors(node):
                # if explored.count(x[0]) == 0:
                fringe.push(x[0])    # x[0] - coordinates of the child-nodes
                temp = completedPath + [x[1]]  # x[1] - actions taken from node to reach child node at x[0]
                pathStack.push(temp)
            # print("Explored nodes: ", explored)

        # The latest path is extracted from the path Stack and added to completed Path(In LIFO)
        if not pathStack.isEmpty():
            completedPath = pathStack.pop()
    return []

    # **Venkatesh Code Ends Here**
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # **Venkatesh Code Start Here**
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    # # Initialization
    # import numpy as np
    from util import Queue
    fringe = Queue()  # The open-set
    fringe.push(problem.getStartState())
    explored = []        # closed set of the visited nodes
    pathQueue = Queue()  # Stores the ways the pacman can go from a node with multiple successors
    completedPath = []
    # node = fringe.pop()

    while not fringe.isEmpty():
        node = fringe.pop()
        # print("The node popped out is: ", node)

        # Goal-test
        if problem.isGoalState(node):
            # print("!GOAL REACHED!")
            return completedPath

        if explored.count(node) == 0:         # check whether new node is present in the closed-set
            explored.append(node)
            for x in problem.getSuccessors(node):
                # if explored.count(x[0]) == 0:
                fringe.push(x[0])   # x[0] - coordinates of the child-nodes
                temp = completedPath + [x[1]]  # x[1] - actions taken from node to reach child node at x[0]
                pathQueue.push(temp)
            # print("Explored nodes: ", explored)

        # The latest path is extracted from the path Queue and added to completed Path (In FIFO)
        if not pathQueue.isEmpty():
           completedPath = pathQueue.pop()
    return []

    # **Venkatesh Code Ends Here**
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    # **Venkatesh Code Start Here**
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    # Initialization
    # import numpy as np
    from util import PriorityQueue
    fringe = PriorityQueue()  # The open-set
    fringe.push(problem.getStartState(), 0)
    explored = []
    pathQueue = PriorityQueue()  # Stores the ways the pacman can go from a node with multiple successors
    completedPath = []
    totalCost = 0

    while not fringe.isEmpty():
        node = fringe.pop()
        # print("The node popped out is: ", node)

        # Goal-test
        if problem.isGoalState(node):
            # print("!GOAL REACHED!")
            return completedPath

        if explored.count(node) == 0:
            explored.append(node)
            for x in problem.getSuccessors(node):
                # print("Node pushed to fringe: ", x[0])
                temp = completedPath + [x[1]]  # x[1] = direction to reach to the child
                tempCost = problem.getCostOfActions(temp)
                pathQueue.push(temp, tempCost)
                fringe.push(x[0], tempCost)  # x[0] - coordinates of the child-nodes
            # print("Explored nodes: ", explored)

        # The latest path is extracted from the path priority Queue and added to completed Path (In min-heap fashion)
        completedPath = pathQueue.pop()
    return []

    # **Venkatesh Code Ends Here**
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    # **Venkatesh Code Start Here**
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    # Initialization
    import numpy as np
    from util import PriorityQueue
    fringe = PriorityQueue()  # The open-set
    fringe.push(problem.getStartState(), 0)
    explored = []
    pathQueue = PriorityQueue()  # Stores the ways the pacman can go from a node with multiple successors
    completedPath = []
    totalCost = 0

    while not fringe.isEmpty():
        node = fringe.pop()
        # print("The node popped out is: ", node)

        # Goal-test
        if problem.isGoalState(node):
            # print("!GOAL REACHED!")
            return completedPath

        if explored.count(node) == 0:
            explored.append(node)
            for x in problem.getSuccessors(node):
                # print("Node pushed to fringe: ", x[0])
                temp = completedPath + [x[1]]  # x[1] = direction to reach to the child
                tempCost = problem.getCostOfActions(temp) + heuristic(x[0], problem)  # f(n) = g(n) + h(n),
                pathQueue.push(temp, tempCost)
                fringe.push(x[0], tempCost)  # x[0] - coordinates of the child-nodes
            # print("Explored nodes: ", explored)

        # The latest path is extracted from the path priority Queue and added to completed Path (In min-heap fashion)
        completedPath = pathQueue.pop()
    return []
    # **Venkatesh Code Ends Here**
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
