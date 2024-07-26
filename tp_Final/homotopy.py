import pyvisgraph as vg
from algorithms import dijkstra, path
from prioritydictionary import priorityDictionary
from graph import DiGraph
import matplotlib.pyplot as plt
from operator import itemgetter

# Função para encontrar os centros de polígonos convexos ou quase convexos
def findCenters(polys):
    def x(elem):
        return elem[0].x
    centers = []
    for i, poly in enumerate(polys):
        _x_list = [vertex.x for vertex in poly]
        _y_list = [vertex.y for vertex in poly]
        _len = len(poly)
        _x = sum(_x_list) / _len - 0.1
        _y = sum(_y_list) / _len 
        centers.append([vg.Point(_x, _y), i+1])
    centers.sort(key=x)
    return centers

# Função para calcular a h-assinatura de uma linha reta entre dois pontos
def Signature(centers, p1, p2):
    signature = []
    _a = (p1.y - p2.y) / (p1.x - p2.x) if p1.x - p2.x != 0 else float("inf")
    _reverse = False

    for center in centers:
        if p1.x <= center[0].x <= p2.x or p2.x <= center[0].x <= p1.x:
            y = _a * (center[0].x - p1.x) + p1.y if _a != float("inf") else None
            if (_a != float("inf") and center[0].y <= y) or (center[0].x == p1.x and (center[0].y < p1.y or center[0].y < p2.y)):
                signature.append(center[1])
                if p1.x > p2.x:
                    signature[-1] *= -1
                    _reverse = True

    if _reverse:
        signature.reverse()
    return signature

# Função para reduzir uma h-assinatura removendo elementos desnecessários
def Reduce(signature):
    if len(signature) <= 1:
        return signature

    reduced = []
    i = 0
    while i < len(signature) - 1:
        if signature[i] == -signature[i + 1] or signature[i] == signature[i + 1]:
            i += 2
        else:
            reduced.append(signature[i])
            i += 1
    if i < len(signature):
        reduced.append(signature[i])
    return Reduce(reduced) if len(reduced) < len(signature) else reduced

# Função para inverter uma h-assinatura
def Invert(signature):
    return [-sig for sig in reversed(signature)]

# Função para calcular o custo de um caminho no grafo
def pathCost(graph, path):
    return sum(graph[path[i]][path[i+1]] for i in range(len(path) - 1))

# Função para calcular a h-assinatura de um caminho no grafo
def pathSignature(graph, path):
    signature = []
    for i in range(len(path) - 1):
        signature += graph.getSignature(path[i], path[i+1])
    return Reduce(signature)

# Função para comparar duas h-assinaturas
def Compare(signature1, signature2):
    if signature1 == signature2:
        return (False, [], [])
    else:
        oneMinusTwo = [label for label in signature1 if label not in signature2]
        twoMinusOne = [label for label in signature2 if label not in signature1]
        return (True, oneMinusTwo, twoMinusOne)

# Implementação de Dijkstra modificada para seguir apenas arestas com assinaturas em hstar
def hdijkstra(graph, node_start, node_end=None, hstar=[]):
    distances = {}
    previous = {}
    Q = priorityDictionary()
    
    for v in graph:
        distances[v] = graph.INFINITY
        previous[v] = graph.UNDEFINED
        Q[v] = graph.INFINITY
    
    distances[node_start] = 0
    Q[node_start] = 0
    
    for v in Q:
        if v == node_end: break
        for u in graph[v]:
            _hsignature = graph.getSignature(v, u)
            if _hsignature and all(label in hstar for label in _hsignature):
                cost_vu = distances[v] + graph[v][u]
                if cost_vu < distances[u]:
                    distances[u] = cost_vu
                    Q[u] = cost_vu
                    previous[u] = v

    if node_end:
        return {'cost': distances[node_end], 'path': path(previous, node_start, node_end)}
    else:
        return distances, previous

# Função para encontrar múltiplos caminhos de menor custo com restrições de h-assinatura
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
            A.insert(-1, newPath)
        current_start -= 1
        if abs(current_start) == len(previous_path['path']) + 1:
            break
        reversePath.append(str(node_start))
        reverseCost += graph[node_start][previous_path['path'][current_start]]
        node_start = previous_path['path'][current_start]

    A = sorted(A, key=itemgetter('cost'))    
    return A[0] if A else None

# Função para simplificar um caminho removendo nós intermediários desnecessários
def SimplifyPath(graph, path, hSignatureStar=[]):
    i = 0
    newPath = [path[i]]
    newCost = 0.0
    while i < len(path) - 2:
        newCost += graph[newPath[-1]][path[i + 1]]
        newPath.append(path[i + 1])
        j = 2
        removedNodes = []
        while i + j < len(path):
            if path[i + j] in graph[path[i]]:
                if graph.getSignature(path[i], path[i + j]) == pathSignature(graph, path[i:(i + j + 1)]):
                    removedNodes.append(newPath[-1])
                    newCost -= graph[newPath[-2]][newPath[-1]]
                    newPath[-1] = path[i + j]
                    newCost += graph[newPath[-2]][newPath[-1]]
                j += 1
            else:
                break
        i += 1
        while path[i] in removedNodes:
            removedNodes.remove(path[i])
            i += 1
            if i >= len(path):
                break
    newPath += path[i + 1:]
    newCost += pathCost(graph, path[i:])
    return {'cost': newCost, 'path': newPath}

# Função para reconstruir um caminho a partir de uma tabela de predecessores
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

# Implementação do algoritmo Yen's K-Shortest Paths com restrições de h-assinatura
def h_ksp_yen(graph, node_start, node_end, max_k=2, hSignatureStar=[], reverse_path=[]):
    # Determinar o caminho mais curto entre os nós de início e fim
    distances, previous = hdijkstra(graph, node_start, None, hSignatureStar)     
    A = [{'cost': distances[node_end], 'path': path(previous, node_start, node_end)}]
    # Inicializa a lista de caminhos potenciais
    B = []
    if not A[0]['path']:
        return A[0]

    if pathSignature(graph, reverse_path + A[0]['path']) == hSignatureStar:
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
            path_spur = path(previous, node_spur, node_end)
            if path_spur:
                path_total = path_root[:-1] + path_spur
                hsign_total = pathSignature(graph, path_total)
                if hsign_total == hSignatureStar:
                    cost_total = pathCost(graph, path_total)
                    B.append({'cost': cost_total, 'path': path_total})
            for edge in edges_removed:
                graph.add_edge(edge[0], edge[1], edge[2], edge[3])
        if not B:
            break
        B.sort(key=lambda x: x['cost'])
        if B[0] in A:
            B.pop(0)
        A.append(B[0])
        B.pop(0)
    return A[-1] if A else None