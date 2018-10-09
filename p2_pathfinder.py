from math import sqrt
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

    queue = [(0, source_box, source_point)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[(source_box, source_point)] = 0

    # The dictionary that will store the backpointers
    prev_points = {}
    prev_points[(source_box, source_point)] = (None, None)

    while queue:
        current_dist, current_box, current_point = heappop(queue)

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
                    heappush(corner_queue, (distance(current_point, (current_box[0], current_box[2])), (current_box[0], current_box[2])))
                if current_box[3] <= adj_box[3]:
                    heappush(corner_queue, (distance(current_point, (current_box[0], current_box[3])), (current_box[0], current_box[3])))
                if adj_box[2] >= current_box[2]:
                    heappush(corner_queue, (distance(current_point, (adj_box[1], adj_box[2])), (adj_box[1], adj_box[2])))
                if adj_box[3] <= current_box[3]:
                    heappush(corner_queue, (distance(current_point, (adj_box[1], adj_box[3])), (adj_box[1], adj_box[3])))
            elif current_box[1] == adj_box[0]:
                if current_box[2] >= adj_box[2]:
                    heappush(corner_queue, (distance(current_point, (current_box[1], current_box[2])), (current_box[1], current_box[2])))
                if current_box[3] <= adj_box[3]:
                    heappush(corner_queue, (distance(current_point, (current_box[1], current_box[3])), (current_box[1], current_box[3])))
                if adj_box[2] >= current_box[2]:
                    heappush(corner_queue, (distance(current_point, (adj_box[0], adj_box[2])), (adj_box[0], adj_box[2])))
                if adj_box[3] <= current_box[3]:
                    heappush(corner_queue, (distance(current_point, (adj_box[0], adj_box[3])), (adj_box[0], adj_box[3])))
            elif current_box[2] == adj_box[3]:
                if current_box[0] >= adj_box[0]:
                    heappush(corner_queue, (distance(current_point, (current_box[0], current_box[2])), (current_box[0], current_box[2])))
                if current_box[1] <= adj_box[1]:
                    heappush(corner_queue, (distance(current_point, (current_box[1], current_box[2])), (current_box[1], current_box[2])))
                if adj_box[0] >= current_box[0]:
                    heappush(corner_queue, (distance(current_point, (adj_box[0], adj_box[3])), (adj_box[0], adj_box[3])))
                if adj_box[1] <= current_box[1]:
                    heappush(corner_queue, (distance(current_point, (adj_box[1], adj_box[3])), (adj_box[1], adj_box[3])))
            elif current_box[3] == adj_box[2]:
                if current_box[0] >= adj_box[0]:
                    heappush(corner_queue, (distance(current_point, (current_box[0], current_box[3])), (current_box[0], current_box[3])))
                if current_box[1] <= adj_box[1]:
                    heappush(corner_queue, (distance(current_point, (current_box[1], current_box[3])), (current_box[1], current_box[3])))
                if adj_box[0] >= current_box[0]:
                    heappush(corner_queue, (distance(current_point, (adj_box[0], adj_box[2])), (adj_box[0], adj_box[2])))
                if adj_box[1] <= current_box[1]:
                    heappush(corner_queue, (distance(current_point, (adj_box[1], adj_box[2])), (adj_box[1], adj_box[2])))
            #enqueue the next box at the nearest corner
            adj_dist, adj_point = heappop(corner_queue)
            pathcost = current_dist + adj_dist
            #if the cost is better than previous
            if (adj_box, adj_point) not in distances or pathcost < distances[(adj_box, adj_point)]:
                distances[(adj_box, adj_point)] = pathcost
                prev_points[(adj_box, adj_point)] = (current_box, current_point)
                heappush(queue, (pathcost, adj_box, adj_point))
    
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
    return sqrt((p2[0]-p1[0])**2 + (p2[1]+p1[1])**2)
