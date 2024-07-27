import homotopy as h
import matplotlib.pyplot as plt
import pyvisgraph as vg
from algorithms import dijkstra
from graph import DiGraph
from pyvisgraph.visible_vertices import edge_distance


def draw(polys, centers=[]):
    # Desenha os obstáculos
    for poly in polys:
        for i in range(len(poly) - 1):
            plt.plot([poly[i].x, poly[i+1].x], [poly[i].y, poly[i+1].y], 'b')
        plt.plot([poly[0].x, poly[-1].x], [poly[0].y, poly[-1].y], 'b')
        if centers:
            plt.plot(centers[polys.index(poly)][0].x, centers[polys.index(poly)][0].y, 'ko')
            plt.plot([centers[polys.index(poly)][0].x, centers[polys.index(poly)][0].x], 
                     [centers[polys.index(poly)][0].y, centers[polys.index(poly)][0].y + 100], 'k:')
            plt.text(centers[polys.index(poly)][0].x + 0.3, centers[polys.index(poly)][0].y, str(centers[polys.index(poly)][1]))
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")

def main():
    # Cria os obstáculos
    polys = [
        [vg.Point(0.0, 6.0), vg.Point(3.0, 6.0), vg.Point(1.5, 9.0)],
        [vg.Point(2.0, 0.0), vg.Point(0.7071*2, 0.7071*2), vg.Point(0.0, 2.0), vg.Point(-0.7071*2, 0.7071*2), vg.Point(-2.0, 0.0), vg.Point(-0.7071*2, -0.7071*2), vg.Point(0.0, -2.0), vg.Point(0.7071*2, -0.7071*2)],
        [vg.Point(5.0, 0.0), vg.Point(7.0, 0.0), vg.Point(7.0, 2.0), vg.Point(5.0, 2.0)],
        [vg.Point(6.0, 4.0), vg.Point(9.0, 4.0), vg.Point(9.0, 7.0), vg.Point(7.5, 8.0), vg.Point(5.0, 4.5)],
        [vg.Point(-3.0*1.5, 3.0*1.5), vg.Point(-6.0*1.5, 2.0*1.5), vg.Point(-6.0*1.5, 6.0*1.5), vg.Point(-2.5*1.5, 6.0*1.5), vg.Point(-2.0*1.5, 1.5*1.5)],
        [vg.Point(-8.0, -4.0), vg.Point(-5.0, -4.0), vg.Point(-6.5, 0.5)],
        [vg.Point(3.0+5.0, -5.0), vg.Point(0.7071*3+5.0, 0.7071*3-5.0), vg.Point(0.0+5.0, 3.0-5.0), vg.Point(-0.7071*3+5.0, 0.7071*3-5.0), vg.Point(-3.0+5.0, -5.0), vg.Point(-0.7071*3+5.0, -0.7071*3-5.0), vg.Point(0.0+5.0, -3.0-5.0), vg.Point(0.7071*3+5.0, -0.7071*3-5.0)],
        [vg.Point(9.1, 0.0), vg.Point(9.0, -1.0), vg.Point(9.8, -1.0)]
    ]

    centers = h.findCenters(polys)

    plt.figure(2)
    draw(polys, centers)	

    # Cria o grafo de visibilidade
    g = vg.VisGraph()  
    g.build(polys)

    # Plota o grafo de visibilidade
    plt.figure(1)
    print("Visibility Graph:")
    #draw(polys)	
    for poly in polys: # Para cada polígono na lista de obstáculos
        for vertex in poly: # Para cada vértice no polígono
            for edge in g.visgraph[vertex]: # Para cada aresta no grafo de visibilidade
                adj = edge.get_adjacent(vertex) # Vértice adjacente
                plt.plot([vertex.x, adj.x], [vertex.y, adj.y], 'b:') # Plota a aresta
            plt.plot(vertex.x, vertex.y, 'ro', markersize=8) # Plota o vértice
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.ginput()

    G = DiGraph()  # Grafo para busca de caminhos
    node = 1
    nodenum = {}  # Dicionário para transformar posições 2D em rótulos de nós em G (números)

    # Cria o grafo com nós
    for poly in polys: # Para cada polígono na lista de obstáculos
        for vertex in poly: # Para cada vértice no polígono
            G.add_node(str(node)) # Adiciona um nó ao grafo
            nodenum[vertex] = str(node) # Associa o vértice ao rótulo do nó
            node += 1 # Incrementa o rótulo do nó

    # Cria as arestas com custos
    for poly in polys:
        for vertex in poly:
            for edge in g.visgraph[vertex]:
                adj = edge.get_adjacent(vertex)
                cost = edge_distance(vertex, adj)
                signature = h.Signature(centers, vertex, adj)
                G.add_edge(nodenum[vertex], nodenum[adj], cost, signature)

    print("Printando o node: ", node)
    # Ponto inicial é o ponto âncora
    start = [[-4.0, 0.0]]	
    print("Definindo o ponto de partida: ", start)
    start_point = vg.Point(start[0][0], start[0][1])

    # Atualiza o mapa de visibilidade
    g.update([start_point], start_point)

    # Atualiza o grafo de busca com o start_point
    G.add_node(str(node))
    nodenum[start_point] = str(node)
    node += 1
    for edge in g.visgraph[start_point]:
        adj = edge.get_adjacent(start_point)
        cost = edge_distance(start_point, adj)
        signature = h.Signature(centers, start_point, adj)
        G.add_edge(nodenum[start_point], nodenum[adj], cost, signature)
        G.add_edge(nodenum[adj], nodenum[start_point], cost, h.Invert(signature))	

    # Plota a posição base
    plt.figure(2)
    print("Base position:")
    plt.plot(start_point.x, start_point.y, 'r*', markersize=15)
    plt.text(start_point.x + 0.4, start_point.y, r'$p_b$', fontsize=14)

    spath = {'cost': 0, 'path': [nodenum[start_point]]}
    hSigk = h.pathSignature(G, spath['path'])

    current_point = start_point
    short_path = []
	
    saved_shortest_paths = []
    accumulated_shortest_path_cost = 0
    saved_robot_paths = []
    accumulated_robot_path_cost = 0

    ###### Loop
    for cont in range(0,5):
        # Leitura da meta
        while 1:
            goal = plt.ginput()
            goal_point = vg.Point(goal[0][0], goal[0][1])			
            if g.point_in_polygon(goal_point) == -1:
                break       

        # Atualiza o mapa de visibilidade
        g.update([goal_point], start_point, goal_point)
        g.update([current_point], start_point, goal_point)
    
        # Atualiza o grafo de busca com o goal_point
        G.add_node(str(node))
        nodenum[goal_point] = str(node)
        node += 1
        for edge in g.visgraph[goal_point]:
            adj = edge.get_adjacent(goal_point)
            cost = edge_distance(goal_point, adj)
            signature = h.Signature(centers, goal_point, adj)
            G.add_edge(nodenum[adj], nodenum[goal_point], cost, h.Invert(signature)) 
            G.add_edge(nodenum[goal_point], nodenum[adj], cost, signature) 

        plt.clf()    
        draw(polys, centers)
        
        # Plota a posição base
        plt.plot(start_point.x, start_point.y, 'r*', markersize=15)
        plt.text(start_point.x + 0.4, start_point.y, r'$p_b$', fontsize=14)

        print("*********** New Motion ***************")

        # Busca o caminho mais curto
        print("Compute the Shortest Path")
        print("Goal Position:", current_point)
        plt.plot(current_point.x, current_point.y, 'rs', markersize=8)
        plt.text(current_point.x + 0.4, current_point.y, r'$p_g$', fontsize=14)
        
        for j in range(0, len(short_path) - 1):
            plt.plot([short_path[j].x, short_path[j+1].x] , [short_path[j].y, short_path[j+1].y], ':g', linewidth=1)
        
        hSigkant = hSigk
        previous_spath = spath
        
        # Calcula o novo caminho mais curto
        spath = dijkstra(G, nodenum[current_point], nodenum[goal_point])
        hSigk = h.pathSignature(G, spath['path'])
        
        # Plote o novo caminho mais curto e o novo goal
        plt.plot(goal_point.x, goal_point.y, 'yo', markersize=10)
        plt.text(goal_point.x + 0.4, goal_point.y, r'$p_g$', fontsize=14)
        short_path = []
        
        for i in range(0, len(spath['path'])):
            for key, value in nodenum.items():
                if value == spath['path'][i]:
                    short_path.append(key)
        
        for j in range(0, len(short_path) - 1):
            plt.plot([short_path[j][0].x, short_path[j+1][0].x], [short_path[j][0].y, short_path[j+1][0].y], 'g', linewidth=1)
        
        print("Current Shortest Path Cost:", spath['cost'])
        accumulated_shortest_path_cost += spath['cost']
        print("Accumulated Shortest Path Cost:", accumulated_shortest_path_cost)

        # Calcula a assinatura desejada
        hSigStar  = h.Reduce(h.Invert(hSigkant) + hSigk)
        
        # Computa o caminho do robo
        k = 20
        shortpath = h.shp(G, previous_spath, nodenum[goal_point], k, hSigStar)
        
        # Se encontrar um caminho
        if shortpath:
            print("Shortest Homotopic Path Found")
            short_path = []
            print("Shortest Homotopic Path Cost:", shortpath['cost'], "-> ".join(shortpath['path']))
            
            for i in range(0, len(shortpath['path'])):
                for key, value in nodenum.items():
                    if value == shortpath['path'][i]:
                        short_path.append(key)
                        
            for j in range(0, len(short_path) - 1):
                plt.plot([short_path[j].x, short_path[j+1].x], [short_path[j].y, short_path[j+1].y], 'r--', linewidth=2)
                
            # Shortest Path
            shortest_local_path = dijkstra(G, previous_spath['path'][-1], nodenum[goal_point])
            temp_path = []
            for i in range(0, len(shortest_local_path['path'])):
                for key, value in nodenum.items():
                    if value == shortest_local_path['path'][i]:
                        temp_path.append(key)
            for j in range(0, len(temp_path) - 1):
                plt.plot([temp_path[j].x, temp_path[j+1].x], [temp_path[j].y, temp_path[j+1].y], 'y', linewidth=2)
                
            saved_shortest_paths.insert(0, temp_path) # Insere o caminho mais curto no início da lista
            accumulated_shortest_path_cost += shortest_local_path['cost']
            
            # Robot Path
            saved_robot_paths.insert(0, shortpath) # salva o caminho sem emaranhados do robô para mostrar
            accumulated_robot_path_cost += shortpath['cost']
        else:
            print("Shortest Homotopic Path Not Found")
            
            # Caminho mais longo
            longestPath = list(reversed(previous_spath['path']))+spath['path'][1:]
            print("Original: ", longestPath)
            simp_path = h.SimplifyPath(G, longestPath, hSigStar)
            _simp_path = []
            for i in range(0, len(simp_path['path'])):
                for key, value in nodenum.items():
                    if value == simp_path['path'][i]:
                        _simp_path.append(key)
                        
            # Shortest Path
            shortest_local_path = dijkstra(G, previous_spath['path'][-1], nodenum[goal_point])
            temp_path = []
            for i in range(0, len(shortest_local_path['path'])):
                for key, value in nodenum.items():
                    if value == shortest_local_path['path'][i]:
                        temp_path.append(key)
            for j in range(0, len(temp_path) - 1):
                plt.plot([temp_path[j].x, temp_path[j+1].x], [temp_path[j].y, temp_path[j+1].y], 'y', linewidth=2)
                
            saved_shortest_paths.insert(0, temp_path)
            accumulated_robot_path_cost += shortest_local_path['cost']
            
            # Simplified longest Path
            for j in range(0, len(_simp_path) - 1):
                plt.plot([_simp_path[j].x, _simp_path[j+1].x], [_simp_path[j].y, _simp_path[j+1].y], 'k--', linewidth=2)
                
            saved_robot_paths.insert(-1, simp_path) # Salava o caminho do robo para mostrar sem emaranhados
            accumulated_robot_path_cost += simp_path['cost']
            
        # Remove o goal_point do grafo
        if current_point != start_point:
            for edges in g.visgraph[current_point]:
                adj = edges.get_adjacent(current_point)
                g.visgraph[adj].remove(vg.Edge(adj, current_point))
            g.visgraph[current_point].clear()
            for key in G._data[nodenum[current_point]].keys():
                G._data[key].pop(nodenum[current_point])
                G._hdata[key].pop(nodenum[current_point])
        
            G._data.pop(nodenum[current_point])
            G._hdata.pop(nodenum[current_point])
            del nodenum[current_point]
            
        # Update current point
        current_point = goal_point
        
    # Plote a configuração final do cabo
    plt.figure(3)
    draw(polys)
    plt.title('Configuração Final do Cabo')
    plt.plot(start_point.x, start_point.y, 'r*', markersize=15)
    plt.text(start_point.x+0.4, start_point.y, '$p_b$', fontsize=14)
    i = len(saved_shortest_paths)
    
    for path in saved_shortest_paths:
        for j in range(0, len(path) - 1):
            plt.plot([path[j].x, path[j+1].x], [path[j].y, path[j+1].y], 'y', linewidth=2)
        plt.plot(path[-1].x, path[-1].y, 'ro', markersize=8)
        plt.text(path[-1].x + 0.3, path[-1].y, '$p_{'+str(i)+'}$', fontsize=14)
        i -= 1
        
    plt.figure(4)
    draw(polys)
    plt.plot(start_point.x, start_point.y, 'r*', markersize=15)
    plt.text(start_point.x+0.4, start_point.y, '$p_b$', fontsize=14)
    i = len(saved_robot_paths)
    
    for path in saved_robot_paths:
        for j in range(0, len(path['path']) - 1):
            plt.plot([path['path'][j].x, path['path'][j+1].x], [path['path'][j].y, path['path'][j+1].y], 'r--', linewidth=2)
        plt.plot(path['path'][-1].x, path['path'][-1].y, 'ro', markersize=8)
        plt.text(path['path'][-1].x + 0.3, path['path'][-1].y, '$p_{'+str(i)+'}$', fontsize=14)
        i -= 1
        
    print("Accumulated Shortest Path Cost:", accumulated_shortest_path_cost)
    print("Accumulated Robot Path Cost:", accumulated_robot_path_cost)
    
    plt.show()

if __name__ == "__main__":
    main()
