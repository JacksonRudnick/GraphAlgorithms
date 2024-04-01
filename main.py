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

# For directional graphs linearize them

# Dijkstra's Algorithm
def dijkstra(graph, start):
	"""Dijkstra's algorithm for shortest path.

	Args:
		graph (Graph): The graph to perform Dijkstra's algorithm on.
		start (hashable): The starting vertex for the search.

	Returns:
		dict: The dictionary of vertices and their shortest paths.
	"""
	visited = set()  # Set to keep track of visited vertices
	queue = [(0, start)]  # Queue to perform Dijkstra's algorithm


# Standard Graph Implementation
def standard_graph():
	"""Returns a standard graph."""
	G = Graph()
	G.add_node("A")
	G.add_node("B")
	G.add_node("C")
	G.add_node("D")
	G.add_node("E")
	G.add_node("F")	
	G.add_node("G")
	G.add_node("H")
	G.add_node("I")
	G.add_node("J")
	G.add_node("K")
	G.add_node("L")
	G.add_node("M")
	G.add_node("N")
	G.add_node("O")
	G.add_node("P")

	G.add_edge("A", "B")
	G.add_edge("A", "F")
	G.add_edge("A", "E")

	G.add_edge("B", "C")
	G.add_edge("B", "F")

	G.add_edge("C", "D")
	G.add_edge("C", "G")

	G.add_edge("D", "G")
	
	G.add_edge("E", "F")
	G.add_edge("E", "I")

	G.add_edge("F", "I")

	G.add_edge("G", "J")

	G.add_edge("H", "K")
	G.add_edge("H", "L")

	G.add_edge("I", "J")
	G.add_edge("I", "M")

	G.add_edge("K", "L")
	G.add_edge("K", "O")

	G.add_edge("L", "P")

	G.add_edge("M", "N")

	return G

# DiGraph Implementation
def di_graph():
	"""Returns a directed graph."""
	G = DiGraph()
	G.add_node("1")
	G.add_node("2")
	G.add_node("3")
	G.add_node("4")
	G.add_node("5")
	G.add_node("6")
	G.add_node("7")
	G.add_node("8")
	G.add_node("9")
	G.add_node("10")
	G.add_node("11")
	G.add_node("12")

	G.add_edge("1", "3")

	G.add_edge("2", "1")

	G.add_edge("3", "2")
	G.add_edge("3", "5")

	G.add_edge("4", "1")
	G.add_edge("4", "2")
	G.add_edge("4", "12")

	G.add_edge("5", "6")
	G.add_edge("5", "8")

	G.add_edge("6", "7")
	G.add_edge("6", "8")

	G.add_edge("7", "10")

	G.add_edge("8", "9")
	G.add_edge("8", "10")

	G.add_edge("9", "5")
	G.add_edge("9", "11")

	G.add_edge("10", "11")
	G.add_edge("10", "9")

	G.add_edge("11", "12")

	return G

# Weighted Graph Implementation
def weighted_graph():
	"""Returns a weighted graph."""
	G = Weighted_Graph()
	G.add_node("A")
	G.add_node("B")
	G.add_node("C")
	G.add_node("D")
	G.add_node("E")
	G.add_node("F")	
	G.add_node("G")
	G.add_node("H")
	G.add_node("I")
	
	G.add_edge("A", "B", 22)
	G.add_edge("A", "C", 9)
	G.add_edge("A", "D", 12)

	G.add_edge("B", "C", 35)
	G.add_edge("B", "F", 36)
	G.add_edge("B", "H", 34)

	G.add_edge("C", "F", 42)
	G.add_edge("C", "E", 65)
	G.add_edge("C", "D", 4)

	G.add_edge("D", "E", 33)
	G.add_edge("D", "I", 30)

	G.add_edge("E", "F", 18)
	G.add_edge("E", "G", 23)

	G.add_edge("F", "G", 39)
	G.add_edge("F", "H", 24)

	G.add_edge("G", "H", 25)
	G.add_edge("G", "I", 21)

	G.add_edge("H", "I", 19)

	return G

# Main Function
if __name__ == "__main__":
	G = standard_graph()

	print(dfs(G, "A"))
	print(dfs(G, "H"))

	print(bfs(G, "A"))
	print(bfs(G, "H"))

	G = di_graph()
	#Do stuff here

	print(dfs(G, "1"))

	G = weighted_graph()
	#Do stuff here