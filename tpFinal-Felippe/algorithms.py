from graph import DiGraph
from prioritydictionary import priorityDictionary

def dijkstra(graph, node_start, node_end=None):
    """
    Applies Dijkstra's algorithm to find the shortest path in a graph.

    Args:
        graph (dict): The graph represented as a dictionary of dictionaries.
                      Each key represents a node, and the corresponding value
                      is a dictionary of adjacent nodes and their edge weights.
        node_start: The starting node for the algorithm.
        node_end: The optional ending node for the algorithm. If specified,
                  the function will return the shortest path from the starting
                  node to the ending node.

    Returns:
        If `node_end` is specified:
            dict: A dictionary with two keys:
                - 'cost': The cost of the shortest path from `node_start` to `node_end`.
                - 'path': A list of nodes representing the shortest path from `node_start` to `node_end`.
        If `node_end` is not specified:
            tuple: A tuple containing two dictionaries:
                - The first dictionary contains the distances from `node_start` to each node in the graph.
                - The second dictionary contains the previous node in the shortest path for each node in the graph.
    """
    distances = {}      
    previous = {}       
    Q = priorityDictionary() # Similar a uma fila de prioridades, mantem os nos a serem visitados ordenados por custo
    
    for v in graph:
        distances[v] = graph.INFINITY
        previous[v] = graph.UNDEFINDED
        Q[v] = graph.INFINITY
    
    distances[node_start] = 0
    Q[node_start] = 0
    
    #print(f"PriorityDictionary na função dijkstra: {Q}")
    for v in Q:
        if v == node_end: break

        for u in graph[v]: # Para cada no u adjacente a v
            cost_vu = distances[v] + graph[v][u] # calcula o custo de v a u
            
            if cost_vu < distances[u]: # Se o custo de v a u for menor que o custo anterior
                distances[u] = cost_vu # Atualiza o custo de u
                Q[u] = cost_vu # Atualiza a fila de prioridades
                previous[u] = v # Atualiza o predecessor de u
            
            #print("v:", v, "u:", u, "cost_vu:", cost_vu, "distances[u]:", distances[u])

    if node_end: # Se o no final foi especificado
        return {'cost': distances[node_end], # Retorna o custo do caminho
                'path': path(previous, node_start, node_end)} # Retorna o caminho
    else:
        return (distances, previous) # Retorna as distancias e os predecessores


def path(previous, node_start, node_end):
    """
    Finds the path from the start node to the end node based on the previous nodes.

    Args:
        previous (dict): A dictionary containing the previous node for each node in the graph.
        node_start: The start node.
        node_end: The end node.

    Returns:
        list: The path from the start node to the end node.

    """
    route = []

    node_curr = node_end    
    while True:
        route.append(node_curr)
        if previous[node_curr] == node_start:
            route.append(node_start)
            break
        elif previous[node_curr] == DiGraph.UNDEFINDED:
            return []
        
        node_curr = previous[node_curr]
    
    route.reverse()
    return route