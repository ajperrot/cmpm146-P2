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

    f_path = []
    b_path = []
    f_boxes = []
    b_boxes = []

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
    f_queue = [(0, 0, source_box, source_point)]
    b_queue = [(0, 0, destination_box, destination_point)]

    # The dictionary that will be returned with the costs
    f_distances = {}
    f_distances[(source_box, source_point)] = 0
    b_distances = {}
    b_distances[(destination_box, destination_point)] = 0

    # The dictionary that will store the backpointers
    f_prev_points = {}
    f_prev_points[(source_box, source_point)] = (None, None)
    b_prev_points = {}
    b_prev_points[(destination_box, destination_point)] = (None, None)

    while f_queue and b_queue:
        f_queue, f_distances, f_prev_points, f_boxes, f_path = aBlood(f_queue, f_distances, f_prev_points, f_boxes, f_path, destination_point, mesh, b_boxes)
        if f_path and b_path:
            break
        b_queue, b_distances, b_prev_points, b_boxes, b_path = aBlood(b_queue, b_distances, b_prev_points, b_boxes, b_path, source_point, mesh, f_boxes)
        if f_path and b_path:
            break

    if not f_path or not b_path:
        raise Exception("No Path!") #no path found by the end of search
    else:
        path = f_path[::-1] + b_path

    
    lines = []
    #fill lines with coordinates of lines connecting all points in path
    for i in range(0, len(path) - 1):
        lines.append( (path[i], path[i+1]) )

    all_boxes = f_boxes + b_boxes
    #return
    #print(path)#test
    return lines, all_boxes

def aBlood(queue, distances, prev_points, boxes, path, goal_point, mesh, other_boxes):
    _, current_dist, current_box, current_point = heappop(queue)

    #add newly visited box to boxes
    if current_box not in boxes:
        boxes.append(current_box)

    # Check if current node is the destination
    if current_box in other_boxes:

        # List containing all boxes from source_box to destination_box
        path = [current_point]

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
            verticalQueue(current_box, adj_box, current_point, goal_point, corner_queue, adj_box)
        elif current_box[BOTTOM] == adj_box[TOP]:
            verticalQueue(adj_box, current_box, current_point, goal_point, corner_queue, adj_box)
        elif current_box[LEFT] == adj_box[RIGHT]:
            horizontalQueue(current_box, adj_box, current_point, goal_point, corner_queue, adj_box)
        elif current_box[RIGHT] == adj_box[LEFT]:
            horizontalQueue(adj_box, current_box, current_point, goal_point, corner_queue, adj_box)
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



def verticalQueue(box1, box2, current, destination, queue, adj):
    if box1[LEFT] >= box2[LEFT]:
        enqueueCorner(box1[TOP], box1[LEFT], current, destination, queue)
    if box1[RIGHT] <= box2[RIGHT]:
        enqueueCorner(box1[TOP], box1[RIGHT], current, destination, queue)
    if box2[LEFT] >= box1[LEFT]:
        enqueueCorner(box2[BOTTOM], box2[LEFT], current, destination, queue)
    if box2[RIGHT] <= box1[RIGHT]:
        enqueueCorner(box2[BOTTOM], box2[RIGHT], current, destination, queue)
    if current[X] >= adj[LEFT] and current[X] <= adj[RIGHT]:
        enqueueCorner(box1[TOP], current[X], current, destination, queue)



def horizontalQueue(box1, box2, current, destination, queue, adj):
    if box1[TOP] >= box2[TOP]:
        enqueueCorner(box1[TOP], box1[LEFT], current, destination, queue)
    if box1[BOTTOM] <= box2[BOTTOM]:
        enqueueCorner(box1[BOTTOM], box1[LEFT], current, destination, queue)
    if box2[TOP] >= box1[TOP]:
        enqueueCorner(box2[TOP], box2[RIGHT], current, destination, queue)
    if box2[BOTTOM] <= box1[BOTTOM]:
        enqueueCorner(box2[BOTTOM], box2[RIGHT], current, destination, queue)
    if current[Y] >= adj[TOP] and current[Y] <= adj[BOTTOM]:
        enqueueCorner(current[Y], box2[RIGHT], current, destination, queue)



def enqueueCorner(adjY, adjX, current, destination, queue):
    adj = (adjY, adjX) #adjacent point
    dist = distance(current, adj) #adjacent distance
    est = dist + distance(adj, destination) #estimated total distance
    heappush(queue, (est, dist, adj))



#calculates euclidean distance between two points
def distance(p1, p2):
    return sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)