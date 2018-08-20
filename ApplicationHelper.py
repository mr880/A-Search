#!usr/bin/python

import os


# This is our heuristic function when calculating Manhattan distance.
def distanceBetween(i, j, x, y):
    return abs(i - x) + abs(j - y)


# This repeats the Manhattan distance when we are checking for Manhattan heuristic, or returns 0 when it's the other
# thing.
def heuristicFunction(i, j, x, y):
    # Manhattan Distance. Submit the file with this not commented out.
    return distanceBetween(i, j, x, y)
    # Return 0 All the time
    # return 0


def testfunction(initI, initJ, goalI, goalJ):
    behold = "I have returned."
    ## First, we make tuples of the inputs.
    tupleA = (initI, initJ)
    tupleB = (goalI, goalJ)
    # print "NEW INPUTS\n"
    # print initI, initJ, goalI, goalJ
    # print "\n"
    # It's important that we keep order in mind with this.
    tupleA = [1, 2]
    tupleB = [3, 4]
    tupleList = [tupleA, tupleB]

    # This is where the real stuff is.

    #This line has it work with the .cpp. Comment the line following it.
    cwd = os.environ['PRACSYS_PATH']
    # cwd = os.getcwd()
    print "PRACSYSPATH"
    print cwd
    # print cwd
    # Go back to the point where prx_core is, then move up to the folder/file where maze is located.
    # cwd = cwd[:-26]
    mazelocale = "/prx_core/launches/test_maze.txt"
    # mazelocale = "launches/test_maze.txt"
    adjustedCWD = cwd + mazelocale
    # print "ADJUSTED CWD"
    # print adjustedCWD
    i = 0;
    fp = open(adjustedCWD, "r", 2000)
    rows = 0
    cols = 0
    matrix = []

    # This is responsible for our read-in.

    with open(adjustedCWD) as fp:
        w = [int(x) for x in next(fp).split()]
        h = [int(x) for x in next(fp).split()]
        array = [[int(x) for x in line.split()] for line in fp]

    fp.close()
    print "FILE LOADED\n"
    rows = h[0];
    cols = w[0];
    # print "MATRIX\n"
    # print rows
    # print cols
    # print array[0][0]

    # We've now generated our array. Now, how do we navigate? Consider the followin gmotions

    # UP MOTION ->      array[i+1][j]
    # DOWN MOTION ->    array[i-1][j]
    # RIGHT MOTION->    array[i][j-1]
    # LEFT MOTION ->    array[i][j-1]
    # WE cannot do a case if i/j +/- 1 < 0 || i/j +/- 1 >maxRow/maxCol

    # A* Search - The Manhattan Heuristic
    # First, our inital points will be our active points.
    aX = initI
    aY = initJ
    cantMove = False
    # Initialize the initial point.
    pointZero = [initI, initJ]
    # Initialize the Path List.
    # truePath = [pointZero]
    truePath = []
    # This is the formula for our goal distances. We find the euclidean distance between the startstate and goal state.
    distanceZero = distanceBetween(initI, initJ, goalI, goalJ)
    pointDict = {}
    expandedNodes = []
    # print len(expandedNodes)
    # print array

    # print distanceZero
    # print "Goal"
    # print goalI, goalJ
    # print array[initI][initJ]
    # print array[goalI][goalJ]
    while not (aX == goalI and aY == goalJ):  # or not cantMove:
        # print aX, aY
        adjacentNodes = {}
        # Left.
        if aX - 1 >= 0 and array[aX - 1][aY] > 0 and [aX - 1, aY] not in expandedNodes:
            nNode = {(aX - 1, aY): heuristicFunction(aX - 1, aY, goalI, goalJ)}
            adjacentNodes.update(nNode)
        # Up.
        if aY + 1 < len(array) and array[aX][aY + 1] > 0 and [aX, aY + 1] not in expandedNodes:
            nNode = {(aX, aY + 1): heuristicFunction(aX, aY + 1, goalI, goalJ)}
            adjacentNodes.update(nNode)
        # Right.
        if aX + 1 < len(array[0]) and array[aX + 1][aY] > 0 and [aX + 1, aY] not in expandedNodes:
            nNode = {(aX + 1, aY): heuristicFunction(aX + 1, aY, goalI, goalJ)}
            adjacentNodes.update(nNode)
        if aY - 1 >= 0 and array[aX][aY - 1] > 0 and [aX, aY - 1] not in expandedNodes:
            nNode = {(aX, aY - 1): heuristicFunction(aX + 1, aY, goalI, goalJ)}
            adjacentNodes.update(nNode)

        if len(adjacentNodes) == 0:  # No valid adjacent nodes
            if len(truePath) == 1:  # Nowhere to back to.
                cantMove = True
                return truePath  # Return the starting point path.
            else:

                backTo = truePath[len(truePath) - 1]  # Back to the previous point in the path.
                if [aX, aY] not in expandedNodes:
                    expandedNodes.append([aX, aY])
                aX = backTo[0]
                aY = backTo[1]
                truePath = truePath[:-1]
        else:  # We have options. Go to the minimum heuristic value and start again if this isn't the goal.
            # print adjacentNodes
            minNode = min(adjacentNodes, key=adjacentNodes.get)
            # print "Chosen Min"
            # print minNode
            if [aX, aY] not in expandedNodes:
                expandedNodes.append([aX, aY])
            truePath.append([aX, aY])
            aX = minNode[0]
            aY = minNode[1]
            # print aX, aY

    # We have converged. Return the truePath.
    truePath.append([goalI, goalJ])
    print "EXPANDED NODES"
    print len(expandedNodes);
    print "PATH"
    print len(truePath)
    print truePath
    return truePath

    # l = [[]]
    # print fp.read()

    return tupleList


def main():
    print "Easy does it, easy does it. They got something to say to no to."
    testfunction(1, 2, 3, 4)
    # First, we acquire the current directory of which the application is being run on.
    # cwd = os.getcwd()
    # # print cwd
    # # Go back to the point where prx_core is, then move up to the folder/file where maze is located.
    # adjustedCWD = cwd[:-26]
    # mazelocale = "launches/test_maze.txt"
    # adjustedCWD = adjustedCWD + mazelocale
    # fp = open(adjustedCWD, "r", 2000)
    # # Possible Error case: The maze file does not exist.
    # ## address here
    #
    # #We parse the file to get rowsxcolumns, instantiating our array.
    # for line in fp:
    #     print line

    ## We instantiat

    # We instantiate our class.


if __name__ == "__main__":
    main()
