import pyvisgraph as vg
from algorithms import dijkstra, path
from prioritydictionary import priorityDictionary
from graph import DiGraph
from operator import itemgetter

def findCenters(polys):  # Assumes polys are convex or almost convex
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

def Signature(centers, p1, p2):  # h-signature for straight lines -> edges
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

def Reduce(signature):  # reduce h-signature
    if len(signature) <= 1:  # Already very reduced
        return signature

    reduced = []
    keep_simplifing = False
    i = 0
    while i < len(signature) - 1:
        if signature[i] == -signature[i + 1]:   # Two equal with opposite signs
            keep_simplifing = True
            i += 2  # Skip two positions
        elif signature[i] == signature[i + 1]:  # Two equal
            keep_simplifing = True
            reduced.append(signature[i])
            i += 2  # Skip two positions
        else:  # Two different
            reduced.append(signature[i])
            i += 1
            
    if keep_simplifing:
        if i < len(signature): 
            reduced.append(signature[i])
        reduced = Reduce(reduced)
    elif not keep_simplifing:
        reduced.append(signature[i])
    
    return reduced

def Invert(signature):  # invert h-signature
    sig = list(signature)
    sig.reverse()
    for i in range(len(sig)):
        sig[i] = -sig[i]
    return sig

def pathCost(graph, path):  # compute the cost of path in the graph
    cost = 0.0
    for i in range(len(path) - 1):
        cost += graph[path[i]][path[i + 1]]
    return cost

def pathSignature(graph, path):  # compute the path signature
    signature = []
    for i in range(len(path) - 1):
        signature += graph.getSignature(path[i], path[i + 1])
    return Reduce(signature)

def Compare(signature1, signature2): # compare two signatures and return the differences
    _oneMinusTwo = []
    _twoMinusOne = []
    if signature1 == signature2:
        _different = False
    else:
        _different = True
        _oneMinusTwo = [label for label in signature1 if label not in signature2]
        _twoMinusOne = [label for label in signature2 if label not in signature1]

    return (_different, _oneMinusTwo, _twoMinusOne)

def hdijkstra(graph, node_start, node_end=None, hstar=[]): # Dijkstra that only follow edges with signature in hstar
    distances = {}      
    previous = {}       
    Q = priorityDictionary()
    
    for v in graph:
        distances[v] = graph.INFINITY
        previous[v] = graph.UNDEFINDED
        Q[v] = graph.INFINITY
    
    distances[node_start] = 0
    Q[node_start] = 0
    
    for v in Q:
        if v == node_end: 
            break

        for u in graph[v]:
            _hsignature = graph.getSignature(v, u)
            if _hsignature != []:
                hasLabel = False        
                for label in _hsignature:
                    if label in hstar:
                        hasLabel = True
                if not hasLabel:
                    continue            
                
            cost_vu = distances[v] + graph[v][u]
            
            if cost_vu < distances[u]:
                distances[u] = cost_vu
                Q[u] = cost_vu
                previous[u] = v

    if node_end:
        return {'cost': distances[node_end], 
                'path': path(previous, node_start, node_end)}
    else:
        return (distances, previous)  

def shp(graph, previous_path, node_end, max_k=2, hSignatureStar=[]):
    current_start = -1    
    node_start = previous_path['path'][current_start]
    reversePath = []
    reverseCost = 0           

    A = []
      
    for k in range(1, max_k):
        newPath = h_ksp_yen(graph, node_start, node_end, 10, hSignatureStar, reversePath)
        
        if newPath['path']:
            if reversePath:
                newPath['path'] = reversePath + newPath['path']
                newPath['cost'] = reverseCost + newPath['cost']

            A.insert(-1, newPath)  # Insert at the bottom

        current_start -= 1
        if abs(current_start) == (len(previous_path['path']) + 1):
            break
        if reversePath:
            reversePath += [str(node_start)]
        else:
            reversePath = [str(node_start)]
        
        reverseCost += graph[node_start][previous_path['path'][current_start]]
        node_start = previous_path['path'][current_start]

    print(f"ksp run {k} times, finding {len(A)} paths!")
    
    if len(A):
        A = sorted(A, key=itemgetter('cost'))    
        return A[0]

    return None

def SimplifyPath(graph, path, hSignatureStar=[]):  
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

def h_ksp_yen(graph, node_start, node_end, max_k=2, hSignatureStar=[], reverse_path=[]):
    distances, previous = hdijkstra(graph, node_start, None, hSignatureStar)        
    
    A = [{'cost': distances[node_end], 
          'path': path(previous, node_start, node_end)}]
    B = []
    
    if not A[0]['path']: 
        return A[0]

    if pathSignature(graph, reverse_path + A[0]['path']) == hSignatureStar:    
        print("ksp found a path in 1 iteration!")
        return A[0]
   
    for k in range(1, max_k):
        for i in range(len(A[-1]['path']) - 1):
            node_spur = A[-1]['path'][i]
            path_root = A[-1]['path'][:i + 1]
            
            edges_removed = []
            for path_k in A:
                curr_path = path_k['path']
                if len(curr_path) > i and path_root == curr_path[:i + 1]:
                    cost = graph.remove_edge(curr_path[i], curr_path[i + 1])
                    _hsignature = graph.getSignature(curr_path[i], curr_path[i + 1])    
                    if cost == -1:
                        continue
                    edges_removed.append([curr_path[i], curr_path[i + 1], cost, _hsignature])

            path_spur = hdijkstra(graph, node_spur, node_end, hSignatureStar)                
                        
            if path_spur['path']:
                path_total = path_root[:-1] + path_spur['path']
                dist_total = distances[node_spur] + path_spur['cost']
                potential_k = {'cost': dist_total, 'path': path_total}
            
                if potential_k not in B:
                    B.append(potential_k)
            
            for edge in edges_removed:
                graph.add_edge(edge[0], edge[1], edge[2], edge[3])
        
        if len(B):
            B = sorted(B, key=itemgetter('cost'))                
            A.append(B[0])
            if pathSignature(graph, reverse_path + A[-1]['path']) == hSignatureStar:
                print(f"ksp found a path in {k + 1} iterations!")
                return A[-1]        
            B.pop(0)
            A = sorted(A, key=itemgetter('cost'))
        else:
            break
    
    print(f"ksp did not find a path after {k + 1} tries!")
    A[0]['path'] = None    
    return A[0]