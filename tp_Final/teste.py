import homotopy as h
import matplotlib.pyplot as plt
import pyvisgraph as vg
from algorithms import dijkstra
from graph import DiGraph
from pyvisgraph.visible_vertices import edge_distance


def draw(polys, centers=[]):
    # Draw the obstacles
    for i in range(len(polys)):
        for j in range(len(polys[i]) - 1):
            plt.plot([polys[i][j].x, polys[i][j+1].x], [polys[i][j].y, polys[i][j+1].y], 'b')
        plt.plot([polys[i][0].x, polys[i][len(polys[i]) - 1].x], [polys[i][0].y, polys[i][len(polys[i]) - 1].y], 'b')
        if centers:
            plt.plot(centers[i][0].x, centers[i][0].y, 'ko')
            plt.plot([centers[i][0].x, centers[i][0].x], [centers[i][0].y, centers[i][0].y + 100], 'k:')
            plt.text(centers[i][0].x + 0.3, centers[i][0].y, str(centers[i][1]))
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")

def main():
    # Criação dos obstáculos
    polys = [
        [vg.Point(0.0, 6.0), vg.Point(3.0, 6.0), vg.Point(1.5, 9.0)],
        [vg.Point(2.0, 0.0), vg.Point(0.7071 * 2, 0.7071 * 2), vg.Point(0.0, 2.0), vg.Point(-0.7071 * 2, 0.7071 * 2),
         vg.Point(-2.0, 0.0), vg.Point(-0.7071 * 2, -0.7071 * 2), vg.Point(0.0, -2.0), vg.Point(0.7071 * 2, -0.7071 * 2)],
        [vg.Point(5.0, 0.0), vg.Point(7.0, 0.0), vg.Point(7.0, 2.0), vg.Point(5.0, 2.0)],
        [vg.Point(6.0, 4.0), vg.Point(9.0, 4.0), vg.Point(9.0, 7.0), vg.Point(7.5, 8.0), vg.Point(5.0, 4.5)],
        [vg.Point(-3.0 * 1.5, 3.0 * 1.5), vg.Point(-6.0 * 1.5, 2.0 * 1.5), vg.Point(-6.0 * 1.5, 6.0 * 1.5),
         vg.Point(-2.5 * 1.5, 6.0 * 1.5), vg.Point(-2.0 * 1.5, 1.5 * 1.5)],
        [vg.Point(-8.0, -4.0), vg.Point(-5.0, -4.0), vg.Point(-6.5, 0.5)],
        [vg.Point(3.0 + 5.0, -5.0), vg.Point(0.7071 * 3 + 5.0, 0.7071 * 3 - 5.0), vg.Point(0.0 + 5.0, 3.0 - 5.0),
         vg.Point(-0.7071 * 3 + 5.0, 0.7071 * 3 - 5.0), vg.Point(-3.0 + 5.0, -5.0),
         vg.Point(-0.7071 * 3 + 5.0, -0.7071 * 3 - 5.0), vg.Point(0.0 + 5.0, -3.0 - 5.0),
         vg.Point(0.7071 * 3 + 5.0, -0.7071 * 3 - 5.0)],
        [vg.Point(9.1, 0.0), vg.Point(9.0, -1.0), vg.Point(9.8, -1.0)]
    ]

    centers = h.findCenters(polys)

    plt.figure(2)
    draw(polys, centers)

    # Criação do grafo de visibilidade
    g = vg.VisGraph()
    g.build(polys)

    # Plot do grafo de visibilidade
    plt.figure(1)
    print("Visibility Graph:")
    for i in range(len(polys)):
        for j in range(len(polys[i])):
            for edges in g.visgraph[polys[i][j]]:
                adj = edges.get_adjacent(polys[i][j])
                plt.plot([polys[i][j].x, adj.x], [polys[i][j].y, adj.y], 'b:')
            plt.plot(polys[i][j].x, polys[i][j].y, 'ro', markersize=8)
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    #plt.ginput()

    # Atualizar o grafo de busca com o ponto inicial
    G = DiGraph()
    node = 1
    nodenum = {}

    # Criação do grafo com nós
    for i in range(len(polys)):
        for j in range(len(polys[i])):
            G.add_node(str(node))
            nodenum[polys[i][j]] = str(node)
            node += 1

    print("Nodenum dictionary:", nodenum)

    # Criação das arestas com custos
    for i in range(len(polys)):
        for j in range(len(polys[i])):
            for edges in g.visgraph[polys[i][j]]:
                adj = edges.get_adjacent(polys[i][j])
                cost = edge_distance(polys[i][j], adj)
                signature = h.Signature(centers, polys[i][j], adj)
                G.add_edge(nodenum[polys[i][j]], nodenum[adj], cost, signature)

    print(node)
    # Definir os pontos inicial e de destino diretamente no código
    start = [-4.0, 0.0]
    start_point = vg.Point(start[0], start[1])
    goal = [10.0, -3.0]  # Defina seu ponto de destino aqui
    goal_point = vg.Point(goal[0], goal[1])

    # Atualizar o mapa de visibilidade
    g.update([start_point, goal_point], start_point)

    # Adicionar o ponto inicial ao grafo
    G.add_node(str(node))
    nodenum[start_point] = str(node)
    node += 1
    for edges in g.visgraph[start_point]:
        adj = edges.get_adjacent(start_point)
        cost = edge_distance(start_point, adj)
        signature = h.Signature(centers, start_point, adj)
        G.add_edge(nodenum[start_point], nodenum[adj], cost, signature)
        G.add_edge(nodenum[adj], nodenum[start_point], cost, h.Invert(signature))

    # Adicionar o ponto de destino ao grafo
    G.add_node(str(node))
    nodenum[goal_point] = str(node)
    node += 1
    for edges in g.visgraph[goal_point]:
        adj = edges.get_adjacent(goal_point)
        cost = edge_distance(goal_point, adj)
        signature = h.Signature(centers, goal_point, adj)
        print("Signature:", signature)
        G.add_edge(nodenum[goal_point], nodenum[adj], cost, signature)
        G.add_edge(nodenum[adj], nodenum[goal_point], cost, h.Invert(signature))

    # Plot da posição inicial e do destino
    plt.figure(2)
    print("Base position and Goal position:")
    plt.plot(start_point.x, start_point.y, 'r*', markersize=15, label="Start")
    plt.plot(goal_point.x, goal_point.y, 'g*', markersize=15, label="Goal")

    # Calculando o caminho mais curto usando Dijkstra
    shortest_path = dijkstra(G, nodenum[start_point], nodenum[goal_point])
    print(f"Priority Dic in main: {G}")
    print("Shortest path from start to goal:", shortest_path)

    # Plotando o caminho mais curto
    path = shortest_path['path']
    path_points = [next(key for key, value in nodenum.items() if value == node) for node in path]
    x_coords = [point.x for point in path_points]
    y_coords = [point.y for point in path_points]
    plt.plot(x_coords, y_coords, 'y--', linewidth=2, label="Path")

    plt.title("Base and Goal Positions with Path")
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
