import pyvisgraph as vg
from pyvisgraph.visible_vertices import edge_distance
import matplotlib.pyplot as plt
from graph import DiGraph
from algorithms import dijkstra
import homotopy as h

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
    for poly in polys:
        for vertex in poly:
            for edge in g.visgraph[vertex]:
                adj = edge.get_adjacent(vertex)				
                plt.plot([vertex.x, adj.x], [vertex.y, adj.y], 'b:')
            plt.plot(vertex.x, vertex.y, 'ro', markersize=8)
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.ginput()

    G = DiGraph()  # Grafo para busca de caminhos
    node = 1
    nodenum = {}  # Dicionário para transformar posições 2D em rótulos de nós em G (números)

    # Cria o grafo com nós
    for poly in polys:
        for vertex in poly:
            G.add_node(str(node))
            nodenum[vertex] = str(node)
            node += 1

    # Cria as arestas com custos
    for poly in polys:
        for vertex in poly:
            for edge in g.visgraph[vertex]:
                adj = edge.get_adjacent(vertex)
                cost = edge_distance(vertex, adj)
                signature = h.Signature(centers, vertex, adj)
                G.add_edge(nodenum[vertex], nodenum[adj], cost, signature)

    print(node)
    # Ponto inicial é o ponto âncora
    start = [[-4.0, 0.0]]	
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
    for _ in range(5):
        # Leitura da meta
        while True:
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

        # Plota a linha de amarração mais curta anterior e o ponto atual
        plt.plot(current_point.x, current_point.y, 'rs', markersize=8)
        plt.text(current_point.x - 0.1, current_point.y - 0.4, r'$p_s$', fontsize=14)
        for point in short_path:
            plt.plot(point[0].x, point[0].y, 'r', linestyle='dotted', linewidth=2.0)

        # Busca o caminho mais curto
        print("Compute the Shortest Path")
        print("Goal Position:")
        plt.plot(goal_point.x, goal_point.y, 'rx', markersize=15)
        plt.text(goal_point.x - 1.0, goal_point.y + 0.4, r'$p_g$', fontsize=14)
        
        spath = dijkstra(G, nodenum[current_point], nodenum[goal_point])
        short_path = []
        for node in spath['path']:
            for key, value in nodenum.items():
                if value == node:
                    short_path.append([key, value])
        
        print("Current Shortest Path Cost:", spath['cost'])
        accumulated_shortest_path_cost += spath['cost']
        print("Accumulated Shortest Path Cost:", accumulated_shortest_path_cost)

        # Plota o caminho mais curto
        for node in short_path:
            plt.plot(node[0].x, node[0].y, 'b.')
            plt.text(node[0].x - 0.3, node[0].y, str(node[1]))

        plt.plot(goal_point.x, goal_point.y, 'rx', markersize=15)
        plt.text(goal_point.x - 1.0, goal_point.y + 0.4, r'$p_g$', fontsize=14)

        saved_shortest_paths.append(spath)

        # Planejamento de Movimento do Robô
        print("Compute the Robot Path")
        rpath = h.shortestPath(G, nodenum[current_point], nodenum[goal_point], hSigk, centers)
        robot_path = []
        for node in rpath['path']:
            for key, value in nodenum.items():
                if value == node:
                    robot_path.append([key, value])

        print("Current Robot Path Cost:", rpath['cost'])
        accumulated_robot_path_cost += rpath['cost']
        print("Accumulated Robot Path Cost:", accumulated_robot_path_cost)

        # Plota o caminho do robô
        for node in robot_path:
            plt.plot(node[0].x, node[0].y, 'k.')
            plt.text(node[0].x + 0.3, node[0].y, str(node[1]))

        hSigk = h.pathSignature(G, rpath['path'])
        saved_robot_paths.append(rpath)

        plt.draw()
        plt.ginput()
        current_point = goal_point

if __name__ == "__main__":
    main()
