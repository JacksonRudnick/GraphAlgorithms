"""
Vertex
String representing a vertex in the graph

Edges
Tuple of (Vertex, Vertex)

Directed Edges
Tuple of (From Vertex, To Vertex)

Weighted Edges
Tuple of (Vertex, Vertex, Weight)
"""

#Importing Libraries
import collections

# Graph Class
class Graph:
	def __init__(self):
		self.nodes = set()
		self.edges = collections.defaultdict(list)

	def add_node(self, value):
		self.nodes.add(value)

	def add_edge(self, from_node, to_node):
		self.edges[from_node].append(to_node)
		self.edges[to_node].append(from_node)

# DiGraph Class
class DiGraph:
	def __init__(self):
		self.nodes = set()
		self.edges = collections.defaultdict(list)
		self.distances = {}

	def add_node(self, value):
		self.nodes.add(value)

	def add_edge(self, from_node, to_node, distance):
		self.edges[from_node].append(to_node)
		self.distances[(from_node, to_node)] = distance

# Weighted Graph Class
class Weighted_Graph:
	def __init__(self):
		self.nodes = set()
		self.edges = collections.defaultdict(list)
		self.weights = {}

	def add_node(self, value):
		self.nodes.add(value)

	def add_edge(self, from_node, to_node, weight):
		self.edges[from_node].append(to_node)
		self.edges[to_node].append(from_node)
		self.weights[(from_node, to_node)] = weight

# DFS
# http://eddmann.com/posts/depth-first-search-and-breadth-first-search-in-python/
def dfs(graph, start):
    visited, stack = set(), [start]
    while stack:
        #print('The stack is:', stack)
        vertex = stack.pop()
        if vertex not in visited:
            visited.add(vertex)
            stack.extend(graph[vertex] - visited)
    return visited


# Dijkstra's Algorithm
# https://gist.github.com/econchick/4666413
def dijkstra(graph, initial):
	visited = {initial: 0}
	path = {}

	nodes = set(graph.nodes)

	while nodes: 
		min_node = None
		for node in nodes:
			if node in visited:
				if min_node is None:
					min_node = node
				elif visited[node] < visited[min_node]:
					min_node = node

		if min_node is None:
			break

		nodes.remove(min_node)
		current_weight = visited[min_node]

		for edge in graph.edges[min_node]:
			weight = current_weight + graph.distance[(min_node, edge)]
			if edge not in visited or weight < visited[edge]:
				visited[edge] = weight
				path[edge] = min_node

	return visited, path

# Main Function
if __name__ == "__main__":
	G = Graph()
	G.add_node("a")
	G.add_node("b")
	G.add_node("c")
	G.add_node("d")

	G.add_edge("a", "b")
	G.add_edge("a", "c")
	G.add_edge("b", "d")

	v = dfs(G, "a")

	print(v)