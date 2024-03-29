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

'''
    ----------------------------------------------------------------------------------------
    Added the below functions to implement the working code of Meet in the Middle algorithm 
    as part of CSE 571 Fall 2022 team project.
    ----------------------------------------------------------------------------------------
'''

# For separating the traversal from start to goal and vice versa
FORWARD = "FORWARD"
BACKWARD = "BACKWARD"


# Custom null heuristic for Meet in the Middle
def mmNullHeuristic(direction, state1, state2, state3):
    return 0

def meetInMiddle(problem, heuristic):

    # if no heuristic is passed them we will use mmNullHeuristic as default heuristic
    if not heuristic or heuristic == nullHeuristic:
        heuristic = mmNullHeuristic

    # Actions represented as directions
    NORTH = 'North'
    SOUTH = 'South'
    EAST = 'East'
    WEST = 'West'

    # Complement direction to convert backward action sequence to forward action sequence
    DIRECTION = {NORTH: SOUTH, EAST: WEST, SOUTH: NORTH, WEST: EAST}

    # Function to get the g value by getting the cost of sequence of actions
    def getGValue(actions):
        return problem.getCostOfActions(actions)

    # Function to get the h value by invoking the heuristic calculation function
    def getHValue(direction, state, goal, lastVisitedNode, heuristic):
        return heuristic(direction, state, goal, lastVisitedNode)

    # Complement action sequence and reverse the order of resulting sequence
    def toggleDirectionForActions(actions):
        return [DIRECTION[x] for x in actions][::-1]

    # To store the middle node location where the both expansion from opposite directions meet.
    MIDDLE_NODE = ()

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

    # Creating the priority queue for storing the priorities based on prF (n) = max(fF (n), 2gF (n)).
    openForwardQueue = util.PriorityQueue() # open set dict{}
    openBackwardQueue = util.PriorityQueue() # open set dict{}

    openForward = {}
    openBackward = {}
    # Closed set{} for forward traversal direction
    closedForward = {}
    # Closed set{} for backward traversal direction
    closedBackward = {} # closed set{}

    startNode = problem.getStartState()
    goalNode = problem.getGoalState()

    # For forward direction, last node visited in the opposite direction and vice versa
    lastNodeF = startNode
    lastNodeB = goalNode

    # List to store the actions taken to reach the particular node
    # initially no action has been taken
    INITIAL_ACTIONS = []

    # Contants for better readability
    G_VALUE = "G_VALUE"
    H_VALUE = "H_VALUE"
    PRIORITY_VALUE = "PRIORITY_VALUE"
    ACTION = "ACTION"

    # List to store the sequence of actions taken to meet in the middle,
    # After merging both the direction's actions by taking the forward direction actions as they are
    # and taking the backward direction actions by complementing and reversing them.
    FINAL_ACTION = []

    # Dictionary to store MetaData for each Node i.e G_VALUE, H_VALUE, PRIORITY_VALUE, ACTION
    forwardNodeMetaData = {}
    backwardNodeMetaData = {}

    # Initializing startNode and goalNode as false, i.e not yet visited
    closedForward[startNode] = False
    closedBackward[goalNode] = False

    # Initializing Forward metaData for startNode
    forwardNodeMetaData[startNode] = {H_VALUE: getHValue(FORWARD, startNode, goalNode, lastNodeB, heuristic),
                                      G_VALUE: getGValue(INITIAL_ACTIONS),
                                      PRIORITY_VALUE: 0,
                                      ACTION: INITIAL_ACTIONS}
    # Add startNode to Forward open set
    openForwardQueue.push(startNode, 0)

    # Initializing metaData for goalNode
    backwardNodeMetaData[goalNode] = {H_VALUE: getHValue(BACKWARD, goalNode, startNode, lastNodeF, heuristic),
                                      G_VALUE: getGValue(INITIAL_ACTIONS),
                                      PRIORITY_VALUE: 0,
                                      ACTION: INITIAL_ACTIONS}
    # Adding goalNode to Backward open set
    openBackwardQueue.push(goalNode, 0)

    # Since all the edge costs in the Pacman domain are 1, epsilon is set as 1 by default.
    epsilon = 1

    """
        while (OpenF ̸= ∅) and (OpenB ̸= ∅) do
    """
    while not openForwardQueue.isEmpty() and not openBackwardQueue.isEmpty():
        # Pop both forward and backward. We'll add it back below whichever is not considered.
        prMinFNode = openForwardQueue.pop()
        prMinBNode = openBackwardQueue.pop()

        # The node with minimum priority is added to the closed set
        closedForward[prMinFNode] = True
        closedBackward[prMinBNode] = True

        # The node with minimum priority is removed from the open set
        openForward[prMinFNode] = False
        openBackward[prMinBNode] = False

        # setting values for respective variables based on respective direction metadata,
        # for better readability
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

        """
            if U ≤max(C,fminF,fminB,gminF +gminB +ε)
            then
                return U
        """
        if U <= max(C, forwardG + forwardH, backwardG + backwardH, forwardG + backwardG + epsilon):
            # To plot heat map, just calling it with goal node to satisfy the condition
            problem.isGoalState(goalNode)

            # If above condition has been satisfied, then the traversals have meet in the middle
            # The following information has been displayed for tracking purposes.
            print("The middle node is: ", MIDDLE_NODE[0])
            print("The cost to expand from the start node till the middle: ", MIDDLE_NODE[1])
            print("The cost to expand from the goal node till the middle: ", MIDDLE_NODE[2])
            return FINAL_ACTION

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
            # Updating the last node visited in the forward direction for Last Node Visited Heuristic
            lastNodeF = prMinFNode

            # Remove the prMinFNode from forward open set
            openForward[prMinFNode] = False

            # Reverting back the closed set and open set for the popped Node from Backward Priority Queue
            openBackward[prMinBNode] = True
            closedForward[prMinFNode] = True

            # Put the backward node back since we went with the forward node
            openBackwardQueue.push(prMinBNode, priorityB)

            """
                for each child c of n do
            """

            # iterate the Successors
            for successor in problem.getSuccessors(prMinFNode):
                c = successor[0] # child node location coordinate
                action = successor[1]  # Action needed to reach child
                cost = successor[2] # Cost associated with the action
                newActionList = list(actionF) + [action] # Append action needed to reach child 'c' to the current node i.e prMinFNode action sequence
                cG = float("inf") # Set g value of the successor as infinite initially

                if c in forwardNodeMetaData:
                    # To get the g value of the successor node
                    cG = forwardNodeMetaData[c][G_VALUE]

                # To get the heuristic value of the successor node
                cH = getHValue(FORWARD, c, goalNode, lastNodeB, heuristic)

                # Calculating the new G value using the g value of prMinFNode and adding the cost to reach child 'c'
                '''
                    gF (c) := gF (n) + cost(n, c)
                '''
                cGNew = forwardG + cost

                '''
                    if c ∈ OpenF ∪ ClosedF and gF (c) ≤ gF (n) + cost(n, c) then
                        continue
                    if c ∈ OpenF ∪ ClosedF then 
                        remove c from OpenF ∪ ClosedF
                '''
                if ((c in openForward and openForward[c]) or (c in closedForward and closedForward[c])) and cG <= cGNew:
                    continue

                if (c in openForward and openForward[c]) or (c in closedForward and closedForward[c]):
                    closedForward[c] = False

                '''
                    add c to OpenF
                '''

                openForward[c] = True
                cF = cH + cGNew
                prC = max(cF, 2 * cGNew)
                openForwardQueue.push(c, prC)
                forwardNodeMetaData[c] = {H_VALUE: cH, G_VALUE: cGNew, PRIORITY_VALUE: prC, ACTION: newActionList}

                """
                    if c ∈ OpenB then
                        U :=min(U,gF(c)+gB(c))
                """
                if c in openBackward and openBackward[c]:
                    U = min(U, cGNew + backwardNodeMetaData[c][G_VALUE])
                    MIDDLE_NODE = (c, cGNew, backwardNodeMetaData[c][G_VALUE])
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
            lastNodeB = prMinBNode
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

                if c in backwardNodeMetaData:
                    cG = backwardNodeMetaData[c][G_VALUE]
                cH = getHValue(BACKWARD, c, startNode, lastNodeF, heuristic)

                cGNew = backwardG + cost

                if ((c in openBackward and openBackward[c]) or (c in closedBackward and closedBackward[c])) and cG <= cGNew:
                    continue

                if (c in openBackward and openBackward[c]) or (c in closedBackward and closedBackward[c]):
                    closedBackward[c] = False

                openBackward[c] = True
                cF = cH + cGNew
                prC = max(cF, 2 * cGNew)
                backwardNodeMetaData[c] = {H_VALUE: cH, G_VALUE: cGNew, PRIORITY_VALUE: prC, ACTION: newActionList}
                openBackwardQueue.push(c, prC)

                """
                    if c ∈ OpenF then
                        U :=min(U,gF(c)+gB(c))
                """

                if c in openForward and openForward[c]:
                    U = min(U, cGNew + forwardNodeMetaData[c][G_VALUE])
                    MIDDLE_NODE = (c, forwardNodeMetaData[c][G_VALUE], cGNew)
                    FINAL_ACTION = forwardNodeMetaData[c][ACTION] + toggleDirectionForActions(backwardNodeMetaData[c][ACTION])

    raise Exception("No solution found")

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
mm = meetInMiddle