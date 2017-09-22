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
import time

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
    myFringe = util.Stack()         # A list of nodes that are to be explored
    start = problem.getStartState()
    explored = []

    if problem.isGoalState(start):  # Check if the start is the goal state
        return []                   # If so, do not move

    myFringe.push(((start),[])) # Push in start to be explored
                                # Store both the coordinates and the an empty
                                #   list, which will hold the path to that specific
                                #   node. The path to start is []

    while not myFringe.isEmpty():   # Run until there are no more nodes to be explored
        node = myFringe.pop()

        if node[0] not in explored:             # As long as the coordinate has not already been explored,
                                                # explore
            if problem.isGoalState(node[0]):    # Check if the coordinate is the goal state
                return node[1]                      # Return path to that coordinate

            for state in problem.getSuccessors(node[0]):                # Get all sucessors
                    myFringe.push(((state[0]), node[1]  + [state[1]]))  # Store into the fringe the coordinate
                                                                        #   Store the path to the sucessor as
                                                                        #   the path of the predecessor + the
                                                                        #   path to the sucessor from that
                                                                        #   predecessor

            explored.append(node[0])                                    # Mark the coordinate as already explored

def breadthFirstSearch(problem):
    myFringe = util.Queue()     # Use a Queue instead for BFS
    start = problem.getStartState()
    explored = []
    nodeCounterPart = []        # Will hold the state
    actionCounterPart = []      # Will hold the directions to a state

    # Very similar to above, but a more thorough algorithm, edit so to work
    #   with the CornersProblem. Thorough, since it stores everything with an
    #   index, to be looked up later

    # For CornersProblem
    cornersExplored = []
    cornerPaths = []

    if problem.isGoalState(start):  # Check if the start is the goal state
        return []                   # If so, do not move

    myFringe.push(start)            # Push in start to be explored

    while not myFringe.isEmpty():   # Run until there are no more nodes to be explored
        node = myFringe.pop()

        if node not in explored:                # As long as the coordinate has not already been explored,
                                                # explore

#----------------------------------------- This Section Wil ONLY run for the CornersProblem -----------------------------------------#
            try:
                if (node[0] in problem.getCorners()) and (node[0] not in cornersExplored):  # Check to se if node is a corner
                                                                                            #   and unchecked
                    if len(cornersExplored) == 0: # If First Corner, map to corner, and then come back to starting position
                        cornerPaths += (actionCounterPart[nodeCounterPart.index(node)])
                        cornerPaths += invertPath(actionCounterPart[nodeCounterPart.index(node)])
                    if len(cornersExplored) == 1:   # Go to previous corner, map back, and go to current corner, map back
                        cornerPaths += (actionCounterPart[nodeCounterPart.index(node)])
                        cornerPaths += invertPath(actionCounterPart[nodeCounterPart.index(node)])
                    if len(cornersExplored) == 2:   # Go to previous corner, map back, and go to current corner, map back
                        cornerPaths += (actionCounterPart[nodeCounterPart.index(node)])
                        cornerPaths += invertPath(actionCounterPart[nodeCounterPart.index(node)])
                    if len(cornersExplored) == 3:   # Go to previous corner, map back, and go to current corner
                        cornerPaths += (actionCounterPart[nodeCounterPart.index(node)])
                        return cornerPaths

                    cornersExplored.append(node[0])
            except Exception:
                pass

#-------------------------------------------------------------------------------------------------------------------------------------#

            if problem.isGoalState(node):       # Check if the coordinate is the goal state
                return actionCounterPart[(nodeCounterPart.index(node))] # Return path to that coordinate

            for state in problem.getSuccessors(node):                # Get all sucessors

                nodeCounterPart.append(state[0])                     # Store future node

                if(node == start):
                    actionCounterPart.append([] + [state[1]])       # Used below also, save the action to go to the
                                                                    #   parent node + the action of the sucessor node
                                                                    #   If we are looking at the parent, its "parent"
                                                                    #   will be an empty node, where the action needed to
                                                                    #   get to it is []
                else:
                    actionCounterPart.append(actionCounterPart[(nodeCounterPart.index(node))]+ [state[1]])

                myFringe.push(state[0])  # Store into the fringe the coordinate


            explored.append(node)                                    # Mark the coordinate as already explored


def uniformCostSearch(problem):
    myFringe = util.PriorityQueue() # Use a priority queue, with priority bieng the cost to get to node
    start = problem.getStartState()
    explored = []


    if problem.isGoalState(start):  # Check if the start is the goal state
        return []                   # If so, do not move

    myFringe.push(((start),[]), 0)  # Push in start to be explored
                                    # Store both the coordinates and the an empty
                                    #   list, which will hold the path to that specific
                                    #   node. The path to start is []

    while not myFringe.isEmpty():   # Run until there are no more nodes to be explored
        node = myFringe.pop()

        if node[0] not in explored:             # As long as the coordinate has not already been explored,
                                                # explore
            if problem.isGoalState(node[0]):    # Check if the coordinate is the goal state
                return node[1]                      # Return path to that coordinate

            for state in problem.getSuccessors(node[0]):                        # Get all sucessors
                    cost = problem.getCostOfActions(node[1] + [state[1]])       # Calculate the cost to get to node
                    myFringe.push(((state[0]), node[1]  + [state[1]]), cost)    # Store into the fringe the coordinate
                                                                                #   Store the path to the sucessor as
                                                                                #   the path of the predecessor + the
                                                                                #   path to the sucessor from that
                                                                                #   predecessor

            explored.append(node[0])                                            # Mark the coordinate as already explored

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    myFringe = util.PriorityQueue() # Use a priority queue, with priority bieng the cost to get to node + the heuristic
    start = problem.getStartState()
    explored = []


    if problem.isGoalState(start):  # Check if the start is the goal state
        return []                   # If so, do not move

    myFringe.push(((start),[]), 0)  # Push in start to be explored
                                    # Store both the coordinates and the an empty
                                    #   list, which will hold the path to that specific
                                    #   node. The path to start is []

    while not myFringe.isEmpty():   # Run until there are no more nodes to be explored
        node = myFringe.pop()

        if node[0] not in explored:             # As long as the coordinate has not already been explored,
                                                # explore
            if problem.isGoalState(node[0]):    # Check if the coordinate is the goal state
                return node[1]                      # Return path to that coordinate

            for state in problem.getSuccessors(node[0]):                        # Get all sucessors
                    cost = problem.getCostOfActions(node[1] + [state[1]]) + heuristic(state[0], problem)    # Calculate the cost to get to node + the heuristic
                    myFringe.push(((state[0]), node[1]  + [state[1]]), cost)    # Store into the fringe the coordinate
                                                                                #   Store the path to the sucessor as
                                                                                #   the path of the predecessor + the
                                                                                #   path to the sucessor from that
                                                                                #   predecessor

            explored.append(node[0])                                            # Mark the coordinate as already explored

# Generates a linear path from start to state
#   Remnant of a past solution, but did not work well with the autograder
def pathGen(state, explored, nodeList):
    path = []
    checked = []
    path.append(state[1])

    xComp = state[0][0]
    yComp = state[0][1]
    for past in reversed(explored):
        if (past[0] == xComp) ^ (past[1] == yComp):
                for row in nodeList:
                    if (xComp, yComp) == row[0]:
                        if (xComp, yComp) not in checked:
                            checked.append((xComp, yComp))
                            path.append(row[1])
                xComp = past[0]
                yComp = past[1]

    return path[::-1]

# Created to be used in conjunction with the CornersProblem
#   Will return the inverse of path, which is it backwards
#   and actions reverse. Will essentially find path back to start
def invertPath(pathway):
    newPath = []
    for action in reversed(pathway):
        if(action == "West"):
            newPath.append("East")
        if(action == "East"):
            newPath.append("West")
        if(action == "North"):
            newPath.append("South")
        if(action == "South"):
            newPath.append("North")
    return newPath

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
