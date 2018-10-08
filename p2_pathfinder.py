#from math import inf, sqrt #carried over from P1, delete if unnecessary
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder (y, x)
        destination_point: the ultimate goal the pathfinder must reach (y, x)
        mesh: pathway constraints the path adheres to
            ['boxes']: list of (y1, y2, x1, x2) tuples representing boxes
            ['adj']: dict mapping boxes to list of adjacent boxes

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = []
    points = {} #fill with the point to travel through in each box (used for edge costs)

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
    """
    THE FOLLOWING IS THE GIVEN P1 DIJKSTRA'S COPIED AND PASTED WITH MINNOR CHANGES
    """
    # The priority queue
    queue = [(0, source_box)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[source_box] = 0

    # The dictionary that will store the backpointers
    backpointers = {}
    backpointers[source_box] = None

    while queue:
        current_dist, current_box = heappop(queue)

        if current_box not in boxes:
            #print(current_box)#test
            boxes.append(current_box) #add newly visited box to boxes

        # Check if current node is the destination
        if current_box == destination_box:

            # List containing all boxes from source_box to destination_box
            path = [current_box]

            # Go backwards from destination_box until the source using backpointers
            # and add all the nodes in the shortest path into a list
            current_back_node = backpointers[current_box]
            while current_back_node is not None:
                path.append(current_back_node)
                current_back_node = backpointers[current_back_node]
            break #finish when destination is found

        # Calculate cost from current note to all the adjacent ones
        for adj_box in mesh['adj'][current_box]:
            adj_box_cost = 1 #replace this with distance between detail points
            pathcost = current_dist + adj_box_cost

            # If the cost is new
            if adj_box not in distances or pathcost < distances[adj_box]:
                distances[adj_box] = pathcost
                backpointers[adj_box] = current_box
                heappush(queue, (pathcost, adj_box))
    
    if not path:
        raise Exception("No Path!")

    lines = []
    #fill lines with coordinates of lines connecting all points in points

    #return
    #print(path)#test
    return path, boxes
