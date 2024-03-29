#Importing Libraries
import collections

# Graph Class
class Graph:
	"""Represents a graph data structure."""

	def __init__(self):
		"""Initializes an empty graph."""
		# Set of vertices
		self.nodes = {}

	def add_node(self, value):
		"""Adds a node to the graph."""
		self.nodes[value] = set()

	def add_edge(self, from_node, to_node):
		"""Adds an edge to the graph between two nodes."""
		self.nodes[from_node].add(to_node)
		"""Adds the opposite edge to complete the graph."""
		self.nodes[to_node].add(from_node)

# DiGraph Class
class DiGraph:
	"""Represents a directed graph data structure."""

	def __init__(self):
		"""Initializes an empty directed graph."""
		self.nodes = {}

	def add_node(self, value):
		"""Adds a node to the directed graph."""
		self.nodes[value] = set()

	def add_edge(self, from_node, to_node):
		"""Adds an edge to the directed graph between two nodes."""
		self.nodes[from_node].add(to_node)

# Weighted Graph Class
class Weighted_Graph:
	"""Represents a weighted graph data structure."""

	def __init__(self):
		"""Initializes an empty weighted graph."""
		self.nodes = {}  # Dictionary to store the nodes of the graph
		self.weights = {}  # Dictionary to store the weights of the edges

	def add_node(self, value):
		"""Adds a node to the weighted graph."""
		self.nodes[value] = set()  # Adds the node to the dictionary

	def add_edge(self, from_node, to_node, weight):
		"""Adds an edge to the weighted graph between two nodes.

		Args:
		    from_node (hashable): The starting node of the edge.
		    to_node (hashable): The ending node of the edge.
		    weight (number): The weight of the edge.
		"""	
		self.nodes[from_node].add(to_node)  # Adds the edge to the graph
		self.nodes[to_node].add(from_node)  # Adds the opposite edge for completeness
		self.weights[(from_node, to_node)] = weight  # Adds the weight of the edge
		self.weights[(to_node, from_node)] = weight  # Adds the weight of the opposite edge

# DFS
def dfs(graph, start):
    """Depth-first search algorithm.

    Args:
        graph (Graph): The graph to perform DFS on.
        start (hashable): The starting vertex for the search.

    Returns:
        set: The set of visited vertices.
    """
    visited = set()  # Set to keep track of visited vertices
    stack = [start]  # Stack to perform DFS

    while stack:
        vertex = stack.pop()  # Pop a vertex from the stack

        # If the vertex is not visited, mark it as visited and add
        # its unvisited neighbors to the stack
        if vertex not in visited:
            visited.add(vertex)
            stack.extend(graph.nodes[vertex] - visited)

    return visited

# BFS
def bfs(graph, start):
	"""Breadth-first search algorithm.

	Args:
	    graph (Graph): The graph to perform BFS on.
	    start (hashable): The starting vertex for the search.

	Returns:
	    list: The list of visited vertices in the order they were visited.
	"""
	visited = set()  # Set to keep track of visited vertices
	queue = [start]  # Queue to perform BFS
	path = []  # List to store the order in which vertices are visited

	while queue:
		vertex = queue.pop(0)  # Pop a vertex from the queue

		# If the vertex is not visited, mark it as visited and add
		# its unvisited neighbors to the queue
		if vertex not in visited:
			visited.add(vertex)
			queue.extend(graph.nodes[vertex] - visited)
			path.append(vertex)

	return path

# Dijkstra's Algorithm


# Main Function
if __name__ == "__main__":
	G = Graph()
	G.add_node("a")
	G.add_node("b")
	G.add_node("c")
	G.add_node("d")
	G.add_node("e")

	G.add_edge("a", "b")
	G.add_edge("a", "c")
	G.add_edge("a", "d")
	G.add_edge("a", "e")

	G.add_edge("b", "c")

	G.add_edge("c", "d")
	G.add_edge("c", "e")

	print(dfs(G, "a"))
	print(bfs(G, "a"))