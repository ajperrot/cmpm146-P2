from math import sqrt
from heapq import heappop, heappush

TOP, BOTTOM, LEFT, RIGHT = 0, 1, 2, 3
Y, X = 0, 1

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh
    Args:
        source_point: starting point of the pathfinder (y, x)
        destination_point: the ultimate goal the pathfinder must reach (y, x)
        mesh: pathway constraints the path adheres to
            ['boxes']: list of (y1, y2, x1, x2) tuples representing boxes
            ['adj']: dict mapping boxes to list of adjacent boxes
            y1 = top, y2 = bottom, x1 = left, x2 = right
    Returns:
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = []
    #boxCorners = {(topLeft),(topRight),(bottomLeft),(bottomRight)}

    #find boxes containing source_point and destination_point and add them to boxes
    source_box = None
    destination_box = None
    for box in mesh['boxes']:
        if source_box == None:
            if box[0] <= source_point[0] and box[1] >= source_point[0] and box[2] <= source_point[1] and box[3] >= source_point[1]:
                source_box = box
                if destination_box != None:
                    break #stop looking as soon as source and destination box are found
        if destination_box == None:
            if box[0] <= destination_point[0] and box[1] >= destination_point[0] and box[2] <= destination_point[1] and box[3] >= destination_point[1]:
                destination_box = box
                if source_box != None:
                    break #stop looking as soon as source and destination box are found
    if source_box == None or destination_box == None:
        raise Exception("No Path!") #making sure both points are in valid boxes


    #implement search from source_point to destination_point, filling boxes, path, and points

    print('y1: ' + str(destination_box[0]))
    print('y2: ' + str(destination_box[1]))
    print('x1: ' + str(destination_box[2]))
    print('x2: ' + str(destination_box[3]))

    queue = [(0, 0, source_box, source_point)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[(source_box, source_point)] = 0

    # The dictionary that will store the backpointers
    prev_points = {}
    prev_points[(source_box, source_point)] = (None, None)

    while queue:
        queue, distances, prev_points, boxes, path = aBlood(queue, distances, prev_points, boxes, path, destination_box, destination_point, mesh)
        if destination_point in path:
            break
        """
        current_est, current_dist, current_box, current_point = heappop(queue)
        #print(current_dist)

        #add newly visited box to boxes
        if current_box not in boxes:
            boxes.append(current_box)

        # Check if current node is the destination
        if current_box == destination_box:

            # List containing all boxes from source_box to destination_box
            path = [destination_point, current_point]

            # Go backwards from destination_box until the source using backpointers
            # and add all the nodes in the shortest path into a list
            next_box, next_point = prev_points[(current_box, current_point)]
            while next_box is not None:
                path.append(next_point)
                next_box, next_point = prev_points[(next_box, next_point)]
            break #finish when destination is found

        # Calculate cost from current node to all the adjacent ones
        for adj_box in mesh['adj'][current_box]:
            #find what the next point will be and load the current point in prev_points
            corner_queue = []
            #entries in corner queue are: ((distace to corner), (y, x))
            #add relevent corners of current_box to consideration
            if current_box[0] == adj_box[1]:
                if current_box[2] >= adj_box[2]:
                    #these next 4 lines could easily be one function call with a few arguments?
                    adj_point = (current_box[0], current_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point)) #unsure of how passing data in here would work
                if current_box[3] <= adj_box[3]:
                    adj_point = (current_box[0], current_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[2] >= current_box[2]:
                    adj_point = (adj_box[1], adj_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[3] <= current_box[3]:
                    adj_point = (adj_box[1], adj_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if current_point[1] >= adj_box[2] and current_point[1] <= adj_box[3]:
                    adj_point = (adj_box[1], current_point[1])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
            elif current_box[1] == adj_box[0]:
                if current_box[2] >= adj_box[2]:
                    adj_point = (current_box[1], current_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if current_box[3] <= adj_box[3]:
                    adj_point = (current_box[1], current_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[2] >= current_box[2]:
                    adj_point = (adj_box[0], adj_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[3] <= current_box[3]:
                    adj_point = (adj_box[0], adj_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if current_point[1] >= adj_box[0] and current_point[1] <= adj_box[3]:
                    adj_point = (adj_box[0], current_point[1])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
            elif current_box[2] == adj_box[3]:
                if current_box[0] >= adj_box[0]:
                    adj_point = (current_box[0], current_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if current_box[1] <= adj_box[1]:
                    adj_point = (current_box[1], current_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[0] >= current_box[0]:
                    adj_point = (adj_box[0], adj_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[1] <= current_box[1]:
                    adj_point = (adj_box[1], adj_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if current_point[0] >= adj_box[0] and current_point[0] <= adj_box[1]:
                    adj_point = (current_point[0], adj_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
            elif current_box[3] == adj_box[2]:
                if current_box[0] >= adj_box[0]:
                    adj_point = (current_box[0], current_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if current_box[1] <= adj_box[1]:
                    adj_point = (current_box[1], current_box[3])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[0] >= current_box[0]:
                    adj_point = (adj_box[0], adj_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if adj_box[1] <= current_box[1]:
                    adj_point = (adj_box[1], adj_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
                if current_point[0] >= adj_box[0] and current_point[0] <= adj_box[1]:
                    adj_point = (current_point[0], adj_box[2])
                    adj_dist = distance(current_point, adj_point)
                    est_dist = adj_dist + distance(adj_point, destination_point)
                    heappush(corner_queue, (est_dist, adj_dist, adj_point))
            #enqueue the next box at the nearest corner
            est_test, adj_dist, adj_point = heappop(corner_queue)
            pathcost = current_dist + adj_dist
            est_cost = pathcost + est_test
            #if the cost is better than previous
            if (adj_box, adj_point) not in distances or pathcost < distances[(adj_box, adj_point)]:
                distances[(adj_box, adj_point)] = pathcost
                prev_points[(adj_box, adj_point)] = (current_box, current_point)
                heappush(queue, (est_cost, pathcost, adj_box, adj_point))
                """
    
    if not path:
        raise Exception("No Path!") #no path found by the end of search
    else:
        path.reverse()
    
    lines = []
    #fill lines with coordinates of lines connecting all points in path
    for i in range(0, len(path) - 1):
        lines.append( (path[i], path[i+1]) )

    #return
    #print(path)#test
    return lines, boxes

#calculates euclidean distance between two points
def distance(p1, p2):
    return sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def aBlood(queue, distances, prev_points, boxes, path, destination_box, destination_point, mesh):
    current_est, current_dist, current_box, current_point = heappop(queue)
    #print(current_dist)

    #add newly visited box to boxes
    if current_box not in boxes:
        boxes.append(current_box)

    # Check if current node is the destination
    if current_box == destination_box:

        # List containing all boxes from source_box to destination_box
        path = [destination_point, current_point]

        # Go backwards from destination_box until the source using backpointers
        # and add all the nodes in the shortest path into a list
        next_box, next_point = prev_points[(current_box, current_point)]
        while next_box is not None:
            path.append(next_point)
            next_box, next_point = prev_points[(next_box, next_point)]
        return (queue, distances, prev_points, boxes, path) #finish when destination is found

    # Calculate cost from current node to all the adjacent ones
    for adj_box in mesh['adj'][current_box]:
        #find what the next point will be and load the current point in prev_points
        corner_queue = []
        #entries in corner queue are: ((distace to corner), (y, x))
        #add relevent corners of current_box to consideration
        if current_box[TOP] == adj_box[BOTTOM]:
            if current_box[LEFT] >= adj_box[LEFT]:
                enqueueCorner(current_box[TOP], current_box[LEFT], current_point, destination_point, corner_queue)
            if current_box[RIGHT] <= adj_box[RIGHT]:
                enqueueCorner(current_box[TOP], current_box[RIGHT], current_point, destination_point, corner_queue)
            if adj_box[LEFT] >= current_box[LEFT]:
                enqueueCorner(adj_box[BOTTOM], adj_box[LEFT], current_point, destination_point, corner_queue)
            if adj_box[RIGHT] <= current_box[RIGHT]:
                enqueueCorner(adj_box[BOTTOM], adj_box[RIGHT], current_point, destination_point, corner_queue)
            if current_point[X] >= adj_box[LEFT] and current_point[X] <= adj_box[RIGHT]:
                enqueueCorner(adj_box[BOTTOM], current_point[X], current_point, destination_point, corner_queue)
        elif current_box[BOTTOM] == adj_box[TOP]:
            if current_box[LEFT] >= adj_box[LEFT]:
                enqueueCorner(current_box[BOTTOM], current_box[LEFT], current_point, destination_point, corner_queue)
            if current_box[RIGHT] <= adj_box[RIGHT]:
                enqueueCorner(current_box[BOTTOM], current_box[RIGHT], current_point, destination_point, corner_queue)
            if adj_box[LEFT] >= current_box[LEFT]:
                enqueueCorner(adj_box[TOP], adj_box[LEFT], current_point, destination_point, corner_queue)
            if adj_box[RIGHT] <= current_box[RIGHT]:
                enqueueCorner(adj_box[TOP], adj_box[RIGHT], current_point, destination_point, corner_queue)
            if current_point[X] >= adj_box[LEFT] and current_point[X] <= adj_box[RIGHT]:
                enqueueCorner(adj_box[TOP], current_point[X], current_point, destination_point, corner_queue)
        elif current_box[LEFT] == adj_box[RIGHT]:
            if current_box[TOP] >= adj_box[TOP]:
                enqueueCorner(current_box[TOP], current_box[LEFT], current_point, destination_point, corner_queue)
            if current_box[BOTTOM] <= adj_box[BOTTOM]:
                enqueueCorner(current_box[BOTTOM], current_box[LEFT], current_point, destination_point, corner_queue)
            if adj_box[TOP] >= current_box[TOP]:
                enqueueCorner(adj_box[TOP], adj_box[RIGHT], current_point, destination_point, corner_queue)
            if adj_box[BOTTOM] <= current_box[BOTTOM]:
                enqueueCorner(adj_box[BOTTOM], adj_box[RIGHT], current_point, destination_point, corner_queue)
            if current_point[Y] >= adj_box[TOP] and current_point[Y] <= adj_box[BOTTOM]:
                enqueueCorner(current_point[Y], adj_box[RIGHT], current_point, destination_point, corner_queue)
        elif current_box[RIGHT] == adj_box[LEFT]:
            if current_box[TOP] >= adj_box[TOP]:
                enqueueCorner(current_box[TOP], current_box[RIGHT], current_point, destination_point, corner_queue)
            if current_box[BOTTOM] <= adj_box[BOTTOM]:
                enqueueCorner(current_box[BOTTOM], current_box[RIGHT], current_point, destination_point, corner_queue)
            if adj_box[TOP] >= current_box[TOP]:
                enqueueCorner(adj_box[TOP], adj_box[LEFT], current_point, destination_point, corner_queue)
            if adj_box[BOTTOM] <= current_box[BOTTOM]:
                enqueueCorner(adj_box[BOTTOM], adj_box[LEFT], current_point, destination_point, corner_queue)
            if current_point[Y] >= adj_box[TOP] and current_point[Y] <= adj_box[BOTTOM]:
                enqueueCorner(current_point[Y], adj_box[LEFT], current_point, destination_point, corner_queue)
        #enqueue the next box at the nearest corner
        est_test, adj_dist, adj_point = heappop(corner_queue)
        pathcost = current_dist + adj_dist
        est_cost = pathcost + est_test
        #if the cost is better than previous
        if (adj_box, adj_point) not in distances or pathcost < distances[(adj_box, adj_point)]:
            distances[(adj_box, adj_point)] = pathcost
            prev_points[(adj_box, adj_point)] = (current_box, current_point)
            heappush(queue, (est_cost, pathcost, adj_box, adj_point))
    return (queue, distances, prev_points, boxes, path)
    
#def verticalCheck(box1, box2, )

def enqueueCorner(adjX, adjY, current, destination, queue):
    adj = (adjX, adjY) #adjacent point
    dist = distance(current, adj) #adjacent distance
    est = dist + distance(adj, destination) #estimated total distance
    heappush(queue, (est, dist, adj))