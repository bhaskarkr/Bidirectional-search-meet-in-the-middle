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
    closed = util.Counter()
    fringe = util.Stack()
    fringe.push((problem.getStartState(), []))
    action_list = []

    while not fringe.isEmpty():
        node_pair = fringe.pop()
        node = node_pair[0]
        action_list = node_pair[1]

        if problem.isGoalState(node):
            return action_list

        if closed[node] == 0:
            closed[node] = 1

            for successor in problem.getSuccessors(node):
                if closed[successor[0]] == 0:
                    fringe.push((successor[0], action_list + [successor[1]]))

    return action_list

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    closed = util.Counter()
    fringe = util.Queue()
    fringe.push((problem.getStartState(), [], 0))
    action_list = []

    while not fringe.isEmpty():
        node_pair = fringe.pop()
        node = node_pair[0]
        action_list = node_pair[1]
        cost = node_pair[2]

        if problem.isGoalState(node):
            print(action_list)
            return action_list

        if closed[node] == 0:
            closed[node] = 1

            for successor in problem.getSuccessors(node):
                if closed[successor[0]] == 0:
                    fringe.push((successor[0], action_list + [successor[1]], successor[2] + cost))

    return action_list

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    closed = util.Counter()
    fringe = util.PriorityQueue()
    fringe.push((problem.getStartState(), [], 0), 0)
    action_list = []

    while not fringe.isEmpty():
        node_pair = fringe.pop()
        node = node_pair[0]
        action_list = node_pair[1]
        cost = node_pair[2]

        if problem.isGoalState(node):
            return action_list

        if closed[node] == 0:
            closed[node] = 1

            for successor in problem.getSuccessors(node):
                if closed[successor[0]] == 0:
                    fringe.push((successor[0], action_list + [successor[1]], successor[2] + cost), successor[2] + cost)

    return action_list

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    closed = util.Counter()
    fringe = util.PriorityQueue()
    fringe.push((problem.getStartState(), [], 0), 0)
    action_list = []

    while not fringe.isEmpty():
        node_pair = fringe.pop()
        node = node_pair[0]
        action_list = node_pair[1]
        cost = node_pair[2]

        if problem.isGoalState(node):
            return action_list

        if closed[node] == 0:
            closed[node] = 1

            for successor in problem.getSuccessors(node):
                if closed[successor[0]] == 0:
                    fringe.push((successor[0], action_list + [successor[1]], successor[2] + cost), successor[2] + heuristic(successor[0], problem) + cost)

    return action_list

def meetInMiddle(problem, heuristic=nullHeuristic):
    openForwardQueue = util.PriorityQueue()
    openBackwardQueue = util.PriorityQueue()
    openForward = util.Counter()
    openBackward = util.Counter()
    closedForward = util.Counter()
    closedBackward = util.Counter()
    forwardNodePriorities = util.Counter()
    backwardNodePriorities = util.Counter()
    U = float("inf")
    startNode = problem.getStartState()
    goalNode = problem.getGoalState() # TODO is this the right function?
    # push node as pair (node, actionList) TODO how to utilize actionList just adding empty list for now
    # priority as pair (priority, gCost) because need to break priority ties by lowest gCost.
    openForwardQueue.push((startNode, []), (0, 0))
    openBackwardQueue.push((goalNode, []), (0, 0))
    forwardNodePriorityTuples[startNode] = (0, 0)
    backwardNodePriorityTuples[goalNode] = (0, 0)
    epsilon = 0 #TODO how do we get smallest cost edge of the problem?

    while not openForward.isEmpty() and not openBackward.isEmpty():
        # Pop both forward and backward. We'll add it back below.
        prMinFNodeTriplet = openForwardQueue.pop()
        prMinBNodeTriplet = openBackwardQueue.pop()
        prMinFNode = prMinFNodeTriplet[0]
        prMinBNode = prMinBNodeTriplet[0]
        forwardPriorityTuple = forwardNodePriorityTuples[prMinFNode]
        backwardPriorityTuple = backwardNodePriorityTuples[prMinFNode]
        forwardG = forwardPriorityTuple[1]
        backwardG = backwardPriorityTuple[1]
        fHeuristic = heuristic(prMinFNode, problem)
        bHeuristic = heuristic(prMinBNode, problem)
        prMinF = max(forwardG + fHeuristic, 2 * forwardG)
        prMinB = max(backwardG + bHeuristic, 2 * backwardG)
        C = min(prMinF, prMinB)

        if U <= max(C, forwardG + fHeuristic, backwardG + bHeuristic, forwardG + backwardG + epsilon):
            return U
        
        if C == prMinF:
            openForward[prMinFNode] = False
            closedForward[prMinFNode] = True
            # Put the backward node back since we went with the forward node
            openBackwardQueue.push(prMinBNodeTriplet, backwardPriorityTuple)
            
            for successor in problem.getSuccessors(prMinFNode):
                c = successor[0]
                cost = successor[2]
                cG = forwardNodePriorityTuples[c][1]

                if (openForward[c] or closedForward[c]) and cG <= (forwardG + cost):
                    continue

                if openForward[c] or closedForward[c]:
                    closedForward[c] = False

                cGNew = forwardG + cost
                openForward[c] = True
                cF = heuristic(c, problem) + cGNew
                prC = max(cF, 2 * cGNew)
                forwardNodePriorityTuples[c] = (prC, cGNew)
                openForwardQueue.push((c, []), (prC, cGNew))

                if openBackward[c]:
                    U = min(U, cGNew + backwardNodePriorityTuples[c][1])
        else:
            openBackward[prMinBNode] = False
            closedBackward[prMinBNode] = True
            # Put the forward node back since we went with the backward node
            openForwardQueue.push(prMinFNodeTriplet, forwardPriorityTuple)

            for successor in problem.getSuccessors(prMinBNode):
                c = successor[0]
                cost = successor[2]
                cG = backwardNodePriorityTuples[c][1]

                if (openBackward[c] or closedBackward[c]) and cG <= (backwardG + cost):
                    continue

                if openBackward[c] or closedBackward[c]:
                    closedBackward[c] = False

                cGNew = backwardG + cost
                openBackward[c] = True
                cF = heuristic(c, problem) + cGNew
                prC = max(cF, 2 * cGNew)
                backwardNodePriorityTuples[c] = (prC, cGNew)
                openBackwardQueue.push((c, []), (prC, cGNew))

                if openForward[c]:
                    U = min(U, cGNew + forwardNodePriorityTuples[c][1])

    return float("inf")


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
mm = meetInMiddle
