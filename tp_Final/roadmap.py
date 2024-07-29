import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.image as mpimg
import networkx as nx
import pyvisgraph as vg
from pyvisgraph.visible_vertices import edge_distance
import matplotlib.pyplot as plt
from graph import DiGraph
from algorithms import dijkstra
import homotopy as h
from math import atan2

# Função para rotacionar em torno do eixo z
def Rz(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

def get_object_position(sim, object_name):
    return sim.getObjectPosition(sim.getObject(object_name), -1)

def get_object_orientation(sim, object_name):
    return sim.getObjectOrientation(sim.getObject(object_name), -1)

fig = plt.figure(figsize=(8,8), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

# Invertendo os valores para visualização (Branco - 0, Preto - 1)
img = 1 - mpimg.imread('cave.png')

# Apenas para garantir que só teremos esses dois valores
threshold = 0.5
img[img > threshold] = 1
img[img<= threshold] = 0

ax.imshow(img, cmap='Greys', origin='upper')

# Dimensões do mapa informado em metros (X, Y)
map_dims = np.array([40, 40]) # Cave 

# Escala Pixel/Metro
sy, sx = img.shape / map_dims

# Tamanho da célula do nosso Grid (em metros)
cell_size = 2

rows, cols = (map_dims / cell_size).astype(int)
grid = np.zeros((rows, cols))

# Preenchendo o Grid
# Cada célula recebe o somatório dos valores dos Pixels
for r in range(rows):
    for c in range(cols):
        
        xi = int(c*cell_size*sx)
        xf = int(xi + cell_size*sx)
        
        yi = int(r*cell_size*sy)
        yf = int(yi + cell_size*sy)
                      
        grid[r, c] = np.sum(img[yi:yf,xi:xf])
        
# Binarizando as células como Ocupadas (1) ou Não-ocupadas (0)       
grid[grid > threshold] = 1
grid[grid<= threshold] = 0        

fig = plt.figure(figsize=(8,8), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

# Plotando Mapa e Células
obj = ax.imshow(img, cmap='Greys', extent=(0, map_dims[1], 0, map_dims[0]), origin='upper')
obj = ax.imshow(grid, cmap='Reds', extent=(0, map_dims[1], 0, map_dims[0]), alpha=.6)

# Plotando as linhas do grid para facilitar a visualização
ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
ax.set_xticks(np.arange(0, map_dims[1]+1, cell_size))
ax.set_yticks(np.arange(0, map_dims[0]+1, cell_size))

def draw(polys, centers=[]):
	# Draw the obstacles
	for i in range(0, len(polys)):
		for j in range(0,len(polys[i])-1):
			plt.plot([polys[i][j].x, polys[i][j+1].x], [polys[i][j].y, polys[i][j+1].y], 'b')
		plt.plot([polys[i][0].x, polys[i][len(polys[i])-1].x], [polys[i][0].y, polys[i][len(polys[i])-1].y], 'b')
		if centers:	
			plt.plot(centers[i][0].x, centers[i][0].y, 'ko')
			plt.plot([centers[i][0].x, centers[i][0].x], [centers[i][0].y, centers[i][0].y+100], 'k:')
			plt.text(centers[i][0].x+0.3, centers[i][0].y, str(centers[i][1]))
	plt.axis('equal')
	plt.axis([0, 40,0, 40])
	plt.xlabel("x (m)")
	plt.ylabel("y (m)")

def find_polygons(edges):
    polys = []
    while edges:
        polygon = []
        start_edge = edges.pop(0)
        current_point = start_edge[0]
        polygon.append(current_point)

        while True:
            # Find the next edge
            for edge in edges:
                if edge[0] == current_point:
                    next_point = edge[1]
                    break
                elif edge[1] == current_point:
                    next_point = edge[0]
                    break
            else:
                # No matching edge found, end of polygon
                break
            
            polygon.append(next_point)
            edges.remove(edge)
            current_point = next_point

            # Check if we have returned to the start point
            if current_point == start_edge[0]:
                break
        
        polys.append(polygon)
    return polys

# Função para verificar se uma célula é um contorno
def is_contour_cell(r, c, grid):
    if grid[r][c] == 1:
        # Verificar se há pelo menos um vizinho livre (0)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                return True
    return False

# Função para adicionar pontos do contorno ao array polys
def add_contour_points(r, c, polys):
    # Coordenadas das arestas da célula (c, r)
    top_left = vg.Point(c * cell_size, map_dims[0] - r * cell_size)
    bottom_left = vg.Point(c * cell_size, map_dims[0] - (r + 1) * cell_size)
    bottom_right = vg.Point((c + 1) * cell_size, map_dims[0] - (r + 1) * cell_size)
    top_right = vg.Point((c + 1) * cell_size, map_dims[0] - r * cell_size)

    # Adicionar pontos de contorno
    if r == 0 or grid[r-1][c] == 0:  # Aresta superior
        polys.append(top_left)
        polys.append(top_right)
    if r == rows-1 or grid[r+1][c] == 0:  # Aresta inferior
        polys.append(bottom_right)
        polys.append(bottom_left)
    if c == 0 or grid[r][c-1] == 0:  # Aresta esquerda
        polys.append(bottom_left)
        polys.append(top_left)
    if c == cols-1 or grid[r][c+1] == 0:  # Aresta direita
        polys.append(top_right)
        polys.append(bottom_right)
        
def orientation(p, q, r):
    """Determina a orientação dos pontos p, q, r"""
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    if val == 0:
        return 0  # Colinear
    return 1 if val > 0 else -1  # Horário ou anti-horário

def graham_scan(points):
    """Encontra o contorno convexo de um conjunto de pontos usando o algoritmo de Graham Scan"""
    # Ordena os pontos por coordenada y (e x em caso de empate)
    points = sorted(points, key=lambda p: (p.y, p.x))

    # Encontra o ponto mais baixo (ou o ponto mais à esquerda em caso de empate)
    pivot = points[0]

    # Ordena os pontos em relação ao ponto pivô
    sorted_points = sorted(points[1:], key=lambda p: (atan2(p.y - pivot.y, p.x - pivot.x), p.x))

    # Constrói o contorno convexo
    hull = [pivot]
    for point in sorted_points:
        while len(hull) > 1 and orientation(hull[-2], hull[-1], point) != -1:
            hull.pop()
        hull.append(point)
    return hull

# Criando vértices em todas as células
G = nx.grid_2d_graph(rows, cols) 
polys = []

# Encontrar e adicionar os pontos de contorno
for r in range(rows):
    for c in range(cols):
        if is_contour_cell(r, c, grid):
            add_contour_points(r, c, polys)

# Converte os pontos de contorno em polígonos
polys = [polys[i:i+2] for i in range(0, len(polys), 2)]

polys = find_polygons(polys)

vert = []

for p in polys:
    print(p)
    hull = graham_scan(p)
    print("Vértices da figura:", hull)
    vert.append(hull)

polys = vert

# Desenha os obstáculos
draw(polys)

# Removendo células que estão em células marcadas com obstáculos
for r in range(rows):
    for c in range(cols):
        if grid[r][c] == 1: 
            G.remove_node((r, c))
    
# Vértices de início e fim
start = (0,0)
end = (rows-1, cols-1)
print("Start:", start)
print("End:", end)

centers = h.findCenters(polys)

plt.figure(2)
plt.title("Centros dos Objetos")
draw(polys, centers)	

# Create the visibility graph
g = vg.VisGraph()  # This is the visibility graph
g.build(polys)

# Plot the visibility graph
plt.figure(1)
print("Visibility Graph:")
for poly in polys: # Para cada polígono na lista de obstáculos
    for vertex in poly: # Para cada vértice no polígono
        for edge in g.visgraph[vertex]: # Para cada aresta no grafo de visibilidade
            adj = edge.get_adjacent(vertex) # Vértice adjacente
            plt.plot([vertex.x, adj.x], [vertex.y, adj.y], 'b:') # Plota a aresta
        plt.plot(vertex.x, vertex.y, 'ro', markersize=8) # Plota o vértice
        
plt.axis('equal')
plt.title("Visibility Graph")
plt.axis([0, 40, 0, 40])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
#plt.ginput()
 
 
G = DiGraph()  # Grafo para busca de caminhos
node = 1
nodenum = {}  # Dicionário para transformar posições 2D em rótulos de nós em G (números)

# Create the Graph with nodes only
for i in range(0, len(polys)):
	for j in range(0,len(polys[i])):
		G.add_node(str(node))
		nodenum[polys[i][j]]=str(node)
		node=node+1

print("Nodenum dictionary:", nodenum)

# Create the edges with costs
for i in range(0, len(polys)):
	for j in range(0,len(polys[i])):
		for edges in g.visgraph[polys[i][j]]:
			adj=edges.get_adjacent(polys[i][j])				
			cost = edge_distance(polys[i][j], adj)
			signature = h.Signature(centers, polys[i][j], adj)
			G.add_edge(nodenum[polys[i][j]], nodenum[adj], cost, signature)

print("Printando o node: ", node)
# Ponto inicial é o ponto âncora
start = [6.0, 38.0]	
print("Definindo o ponto de partida: ", start)
start_point = vg.Point(start[0], start[1])
goal = [10.0, 30.0]  # Defina seu ponto de destino aqui
goal_point = vg.Point(goal[0], goal[1])

# Update the visibility map
g.update([start_point, goal_point], start_point)

# Update the search graph with the start_point
G.add_node(str(node))
nodenum[start_point]=str(node)
node = node + 1
for edges in g.visgraph[start_point]:
	adj=edges.get_adjacent(start_point)
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
    G.add_edge(nodenum[goal_point], nodenum[adj], cost, signature)
    G.add_edge(nodenum[adj], nodenum[goal_point], cost, h.Invert(signature)) 
 
# Plot the base position
plt.figure(2)
print("Base position and Goal position:")
plt.plot(start_point.x, start_point.y, 'r*', markersize=15, label="Start")
plt.plot(goal_point.x, goal_point.y, 'g*', markersize=15, label="Goal")

# Calculando o caminho mais curto usando Dijkstra
shortest_path = dijkstra(G, nodenum[start_point], nodenum[goal_point])
print("Shortest path from start to goal:", shortest_path)

# Plotando o caminho mais curto
path = shortest_path['path']
path_points = [next(key for key, value in nodenum.items() if value == node) for node in path]
x_coords = [point.x for point in path_points]
y_coords = [point.y for point in path_points]

plt.plot(x_coords, y_coords, 'y--', linewidth=2, label="Path")

plt.title("Base and Goal Positions with Path")
plt.axis('equal')
plt.axis([0, 40, 0, 40])
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend()
plt.show()