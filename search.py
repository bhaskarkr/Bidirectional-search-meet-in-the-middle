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

def forwardHeuristic(position, problem):
    return abs(position[0] - problem.startState[0]) + abs(position[1] - problem.startState[1])

def backwardHeuristic(position, problem):
    return abs(position[0] - problem.goal[0]) + abs(position[1] - problem.goal[1])

def meetInMiddle(problem):

    NORTH = 'North'
    SOUTH = 'South'
    EAST = 'East'
    WEST = 'West'

    DIRECTION = {NORTH: SOUTH, EAST: WEST, SOUTH: NORTH, WEST: EAST}

    def getGValue(actions):
        return problem.getCostOfActions(actions)

    def getHValue(state, heuristic):
        # return 0
        return heuristic(state, problem)

    def getFValue(state, actions, heuristic):
        return getGValue(actions) + getHValue(state, heuristic)

    def toggleDirectionForActions(actions):
        return [DIRECTION[x] for x in actions][::-1]


    """
        CheckList:
        ----------
        gF (start) := 0  - Done
        gB (goal) := 0   - Done
        OpenF := {start} - Done
        OpenB := {goal}  - Done
        U := ∞
    """
    U = float("inf")

    openForwardQueue = util.PriorityQueue() # open set dict{}
    openBackwardQueue = util.PriorityQueue() # open set dict{}

    openForward = util.Counter()
    openBackward = util.Counter()
    closedForward = util.Counter() # closed set{}
    closedBackward = util.Counter() # closed set{}

    startNode = problem.getStartState()
    goalNode = problem.getGoalState()

    INITIAL_ACTIONS = []

    G_VALUE = "G_VALUE"
    H_VALUE = "H_VALUE"
    PRIORITY_VALUE = "PRIORITY_VALUE"
    ACTION = "ACTION"

    FINAL_ACTION = []

    forwardNodeMetaData = util.Counter()
    backwardNodeMetaData = util.Counter()

    closedForward[startNode] = False # initially no action has been taken
    closedBackward[goalNode] = False # initially no action has been taken

    forwardNodeMetaData[startNode] = {H_VALUE: getHValue(startNode, forwardHeuristic),
                                      G_VALUE: getGValue(INITIAL_ACTIONS),
                                      PRIORITY_VALUE: 0,
                                      ACTION: INITIAL_ACTIONS}
    openForwardQueue.push(startNode, 0)

    backwardNodeMetaData[goalNode] = {H_VALUE: getHValue(goalNode, backwardHeuristic),
                                      G_VALUE: getGValue(INITIAL_ACTIONS),
                                      PRIORITY_VALUE: 0,
                                      ACTION: INITIAL_ACTIONS}
    openBackwardQueue.push(goalNode, 0)

    epsilon = 0 #TODO how do we get smallest cost edge of the problem?

    """
        while (OpenF ̸= ∅) and (OpenB ̸= ∅) do
    """
    while not openForwardQueue.isEmpty() and not openBackwardQueue.isEmpty():
        # Pop both forward and backward. We'll add it back below.
        prMinFNode = openForwardQueue.pop()
        prMinBNode = openBackwardQueue.pop()
        closedForward[prMinFNode] = True
        closedBackward[prMinBNode] = True
        openForward[prMinFNode] = False
        openBackward[prMinBNode] = False
        forwardG = forwardNodeMetaData[prMinFNode][G_VALUE]
        backwardG = backwardNodeMetaData[prMinBNode][G_VALUE]
        forwardH = forwardNodeMetaData[prMinFNode][H_VALUE]
        backwardH = backwardNodeMetaData[prMinBNode][H_VALUE]
        priorityF = forwardNodeMetaData[prMinFNode][PRIORITY_VALUE]
        priorityB = backwardNodeMetaData[prMinBNode][PRIORITY_VALUE]
        actionF = forwardNodeMetaData[prMinFNode][ACTION]
        actionB = backwardNodeMetaData[prMinBNode][ACTION]
        """
            prF (n) = max(fF (n), 2gF (n)).
        """
        prMinF = max(forwardG + forwardH, 2 * forwardG)
        prMinB = max(backwardG + backwardH, 2 * backwardG)
        """
            C := min(prminF , prminB)
        """
        C = min(prMinF, prMinB)
        print(prMinFNode, prMinBNode)

        """
            if U ≤max(C,fminF,fminB,gminF +gminB +ε)
            then
                return U
        """
        if U <= max(C, forwardG + forwardH, backwardG + backwardH, forwardG + backwardG + epsilon):
            return FINAL_ACTION # need to see how to return actions

        """
            if C = prminF then
        """
        if C == prMinF:
            """
                Expand in the forward direction
                choose n ∈ OpenF for which prF (n) = prminF
                and gF (n) is minimum
                move n from OpenF to ClosedF
            """
            openForward[prMinFNode] = False
            openBackward[prMinBNode] = True
            closedForward[prMinFNode] = True
            # Put the backward node back since we went with the forward node
            openBackwardQueue.push(prMinBNode, priorityB)

            """
                for each child c of n do
                if c ∈ OpenF ∪ ClosedF and gF (c) ≤ gF (n) + cost(n, c) then
                    continue
                if c ∈ OpenF ∪ ClosedF then 
                    removec from OpenF ∪ ClosedF
                gF (c) := gF (n) + cost(n, c)
                add c to OpenF
                if c ∈ OpenB then
                    U :=min(U,gF(c)+gB(c))
            """

            for successor in problem.getSuccessors(prMinFNode):
                c = successor[0]
                action = successor[1]
                cost = successor[2]
                newActionList = list(actionF) + [action]
                cG = float("inf")

                if cG in forwardNodeMetaData:
                    cG = forwardNodeMetaData[cG][G_VALUE]
                cH = getHValue(c, forwardHeuristic)

                cGNew = forwardG + cost

                if (openForward[c] or closedForward[c]):
                    continue

                # if openForward[c] or closedForward[c]:
                #     closedForward[c] = False

                openForward[c] = True
                cF = cH + cGNew
                prC = max(cF, 2 * cGNew)
                openForwardQueue.push(c, prC)
                forwardNodeMetaData[c] = {H_VALUE: cH, G_VALUE: cGNew, PRIORITY_VALUE: prC, ACTION: newActionList}

                """
                    if c ∈ OpenB then
                        U :=min(U,gF(c)+gB(c))
                """

                if openBackward[c]:
                    U = min(U, cGNew + backwardNodeMetaData[c][G_VALUE])
                    FINAL_ACTION = forwardNodeMetaData[c][ACTION] + toggleDirectionForActions(backwardNodeMetaData[c][ACTION])
        else:
            """
                    Analogously for the backward  direction
            """
            """
                Expand in the backward direction
                choose n ∈ OpenF for which prB (n) = prminB
                and gB (n) is minimum
                move n from OpenB to ClosedB
            """
            openBackward[prMinBNode] = False
            openForward[prMinFNode] = True
            closedBackward[prMinBNode] = True
            # Put the backward node back since we went with the forward node
            openForwardQueue.push(prMinFNode, priorityF)

            """
                for each child c of n do
                if c ∈ OpenB ∪ ClosedB and gB (c) ≤ gB (n) + cost(n, c) then
                    continue
                if c ∈ OpenB ∪ ClosedB then 
                    remove c from OpenB ∪ ClosedB
                gB (c) := gB (n) + cost(n, c)
                add c to OpenB
            """

            for successor in problem.getSuccessors(prMinBNode):
                c = successor[0]
                action = successor[1]
                cost = successor[2]
                newActionList = list(actionB) + [action]
                cG = float("inf")

                if cG in backwardNodeMetaData:
                    cG = backwardNodeMetaData[cG][G_VALUE]
                cH = getHValue(c, backwardHeuristic)

                cGNew = backwardG + cost

                if (openBackward[c] or closedBackward[c]): # and cG <= cGNew
                    continue

                # if openBackward[c] or closedBackward[c]:
                #     closedBackward[c] = False

                openBackward[c] = True
                cB = cH + cGNew
                prC = max(cB, 2 * cGNew)
                backwardNodeMetaData[c] = {H_VALUE: cH, G_VALUE: cGNew, PRIORITY_VALUE: prC, ACTION: newActionList}
                openBackwardQueue.push(c, prC)

                """
                    if c ∈ OpenF then
                        U :=min(U,gF(c)+gB(c))
                """

                if openForward[c]:
                    U = min(U, cGNew + forwardNodeMetaData[c][G_VALUE])
                    FINAL_ACTION = forwardNodeMetaData[c][ACTION] + toggleDirectionForActions(backwardNodeMetaData[c][ACTION])
    return FINAL_ACTION

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
mm = meetInMiddle
