import pyvisgraph as vg
from algorithms import dijkstra, path
from prioritydictionary import priorityDictionary
from graph import DiGraph
from operator import itemgetter

def findCenters(polys):  
    """
    Finds the centers of the given polygons.

    Args:
        polys (list): A list of polygons, where each polygon is represented as a list of vertices.

    Returns:
        list: A list of centers, where each center is represented as a list containing a Point object and the index of the polygon.
    """
    centers = []
    for i in range(len(polys)):
        _x_list = [vertex.x for vertex in polys[i]]
        _y_list = [vertex.y for vertex in polys[i]]
        _len = len(polys[i])
        _x = sum(_x_list) / _len - 0.1
        _y = sum(_y_list) / _len 
        centers.append([vg.Point(_x, _y), i + 1])
    centers.sort(key=lambda elem: elem[0].x)
    
    return centers

def Signature(centers, p1, p2):
    """
    Calculates the h-signature for straight lines (edges) between two points.

    Parameters:
    - centers (list): List of centers.
    - p1 (Point): First point.
    - p2 (Point): Second point.

    Returns:
    - signature (list): List of h-signature values.
    """
    signature = []
    if p1.x - p2.x != 0.0:  
        _a = (p1.y - p2.y) / (p1.x - p2.x)
    else:
        _a = float("inf")  # Vertical edge
        
    _reverse = False
    for center in centers:
        if center[0].x >= p1.x and center[0].x <= p2.x:
            if _a != float("inf"):
                y = _a * (center[0].x - p1.x) + p1.y
                if center[0].y <= y:
                    signature.append(center[1])
            elif center[0].x == p1.x and (center[0].y < p1.y or center[0].y < p2.y):  # vertical edges
                signature.append(center[1])            
        elif center[0].x >= p2.x and center[0].x <= p1.x:
            y = _a * (center[0].x - p1.x) + p1.y
            if center[0].y <= y:
                signature.append(-center[1])
                _reverse = True

    if _reverse:
        signature.reverse()    
    return signature

def Reduce(signature):
    """
    Reduce the given h-signature by removing consecutive pairs of elements that satisfy certain conditions.

    Args:
        signature (list): The h-signature to be reduced.

    Returns:
        list: The reduced h-signature.

    """
    if len(signature) <= 1:
        return signature

    reduced = []
    keep_simplifing = False
    i = 0
    while i < len(signature) - 1:
        if signature[i] == -signature[i + 1]:
            keep_simplifing = True
            i += 2
        elif signature[i] == signature[i + 1]:
            keep_simplifing = True
            reduced.append(signature[i])
            i += 2
        else:
            reduced.append(signature[i])
            i += 1
            
    if keep_simplifing:
        if i < len(signature): 
            reduced.append(signature[i])
        reduced = Reduce(reduced)
    elif not keep_simplifing:
        reduced.append(signature[i])
    
    return reduced

def Invert(signature):
    """
    Inverts the given h-signature.

    Args:
        signature (list): The h-signature to be inverted.

    Returns:
        list: The inverted h-signature.
    """
    sig = list(signature)
    sig.reverse()
    for i in range(len(sig)):
        sig[i] = -sig[i]
    return sig

def pathCost(graph, path):
    """
    Compute the cost of a path in the graph.

    Parameters:
    graph (dict): A dictionary representing the graph.
    path (list): A list of nodes representing the path.

    Returns:
    float: The cost of the path in the graph.
    """
    cost = 0.0
    for i in range(len(path) - 1):
        cost += graph[path[i]][path[i + 1]]
    return cost

def pathSignature(graph, path):
    """
    Compute the path signature.

    Args:
        graph: The graph object representing the graph.
        path: The list of nodes representing the path.

    Returns:
        The reduced path signature.

    """
    signature = []
    for i in range(len(path) - 1):
        signature += graph.getSignature(path[i], path[i + 1])
    return Reduce(signature)

def Compare(signature1, signature2):
    """
    Compare two signatures and return the differences.

    Args:
        signature1 (list): The first signature to compare.
        signature2 (list): The second signature to compare.

    Returns:
        tuple: A tuple containing the following elements:
            - different (bool): True if the signatures are different, False otherwise.
            - oneMinusTwo (list): The elements present in signature1 but not in signature2.
            - twoMinusOne (list): The elements present in signature2 but not in signature1.
    """
    _oneMinusTwo = []
    _twoMinusOne = []
    if signature1 == signature2:
        _different = False
    else:
        _different = True
        _oneMinusTwo = [label for label in signature1 if label not in signature2]
        _twoMinusOne = [label for label in signature2 if label not in signature1]

    return (_different, _oneMinusTwo, _twoMinusOne)


def SimplifyPath(graph, path, hSignatureStar=[]):
    """
    Simplifies a given path in a graph by removing unnecessary nodes.

    Args:
        graph (dict): The graph represented as a dictionary where the keys are nodes and the values are dictionaries
                      representing the edges and their corresponding costs.
        path (list): The path to be simplified, represented as a list of nodes.
        hSignatureStar (list, optional): A list representing the h-signature star. Defaults to an empty list.

    Returns:
        dict: A dictionary containing the simplified path and its cost. The dictionary has the following keys:
              - 'cost': The cost of the simplified path.
              - 'path': The simplified path represented as a list of nodes.
    """
    i = 0
    newPath = [path[i]]
    newCost = 0.0
    while i < (len(path) - 2):
        newCost += graph[newPath[-1]][path[i + 1]]
        newPath += [path[i + 1]]
        j = 2
        removedNodes = []
        while (i + j) < len(path):
            if path[i + j] in graph[path[i]]: 
                if graph.getSignature(path[i], path[i + j]) == pathSignature(graph, path[i:(i + j + 1)]):
                    removedNodes += [newPath[-1]]             # store the element to be removed
                    newCost -= graph[newPath[-2]][newPath[-1]]    # remove the cost related to this element
                    newPath[-1] = path[i + j]                # replace the last element by the new one
                    newCost += graph[newPath[-2]][newPath[-1]]  # update the cost
                j += 1
            else:
                break
        i += 1
        while path[i] in removedNodes:     # If the node i was removed, skip to the next node
            removedNodes.remove(path[i])
            i += 1
            if i > len(path):
                break
    newPath += path[i + 1:]
    newCost += pathCost(graph, path[i:]) 
    
    return {'cost': newCost, 'path': newPath}

def path(previous, node_start, node_end):
    """
    Finds the path from the start node to the end node based on the previous nodes.

    Args:
        previous (dict): A dictionary containing the previous node for each node in the graph.
        node_start: The start node.
        node_end: The end node.

    Returns:
        list: The path from the start node to the end node, including both nodes.

    """
    route = []

    node_curr = node_end    
    while True:
        route.append(node_curr)
        if previous[node_curr] == node_start:
            route.append(node_start)
            break
        elif previous[node_curr] == DiGraph.UNDEFINED:
            return []
        
        node_curr = previous[node_curr]
    
    route.reverse()
    return route
