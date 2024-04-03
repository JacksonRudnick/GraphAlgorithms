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
		self.nodes = set()  # Dictionary to store the nodes of the graph
		self.edges = {}  # Dictionary to store the edges of the graph
		self.weights = {}  # Dictionary to store the weights of the edges

	def add_node(self, value):
		"""Adds a node to the weighted graph."""
		self.nodes.add(value)   # Adds the node to the dictionary
		self.edges[value] = set()  # Adds the node to the edges dictionary

	def add_edge(self, from_node, to_node, weight):
		"""Adds an edge to the weighted graph between two nodes.

		Args:
			from_node (hashable): The starting node of the edge.
			to_node (hashable): The ending node of the edge.
			weight (number): The weight of the edge.
		"""	
		self.edges[from_node].add(to_node)  # Add 'to_node' to 'from_node's adjacent nodes
		self.edges[to_node].add(from_node)  # Add 'from_node' to 'to_node's adjacent nodes
		self.weights[(from_node, to_node)] = weight  # Adds the weight of the edge
		self.weights[(to_node, from_node)] = weight  # Adds the weight of the opposite edge

# DFS
def dfs(graph, start):
	"""Depth-first search algorithm.

	Args:
		graph (Graph): The graph to perform DFS on.
		start (hashable): The starting vertex for the search.

	Returns:
		list: The list of visited vertices.
	"""
	visited = []  # List to keep track of visited vertices
	stack = [start]  # Stack to perform DFS
	nodes = list(graph.nodes.keys())
	
	while nodes:
		while stack:
			vertex = stack.pop()  # Pop a vertex from the stack

			# If the vertex is not visited, mark it as visited and add
			# its unvisited neighbors to the stack
			if vertex not in visited:
				nodes.remove(vertex)
				visited.append(vertex)
				for neighbor in graph.nodes[vertex] - set(visited):
					stack.append(neighbor)
		if nodes:
			stack.append(nodes[0])

	return visited

def dfs_search(graph, start, end):
	"""Depth-first search algorithm.

	Args:
		graph (Graph): The graph to perform DFS on.
		start (hashable): The starting vertex for the search.
		end (hashable): The ending vertex for the search.

	Returns:
		list: The list of visited vertices.
	"""
	stack = [start]  # Stack to perform DFS
	visited = set()  # Set to keep track of visited vertices

	while stack:
		vertex = stack.pop()
		if vertex not in visited:  # If vertex is not visited
			if vertex == end:  # If vertex is the end vertex
				return True  # Return the path from start to end
			visited.add(vertex)  # Mark vertex as visited
			for neighbor in graph.nodes[vertex] - visited:  # Add unvisited neighbors to the stack
				stack.append(neighbor)

	return False

# BFS
def bfs(graph, start):
	"""Breadth-first search algorithm.

	Args:
		graph (Graph): The graph to perform BFS on.
		start (hashable): The starting vertex for the search.

	Returns:
		list: The list of visited vertices in the order they were visited.
	"""
	visited = []  # Set to keep track of visited vertices
	queue = [start]  # Queue to perform BFS
	nodes = list(graph.nodes.keys())

	while nodes:
		while queue:
			vertex = queue.pop(0)  # Pop a vertex from the queue

			# If the vertex is not visited, mark it as visited and add
			# its unvisited neighbors to the queue
			if vertex not in visited:
				nodes.remove(vertex)
				visited.append(vertex)
				queue.extend(graph.nodes[vertex] - set(visited))
		if nodes:
			queue.append(nodes[0])

	return visited

# Dijkstra's Algorithm
def dijkstra(graph, start):
	"""Dijkstra's algorithm for finding the shortest path from a given start node to all other nodes in a graph.

	Args:
		graph (Graph): The graph to perform Dijkstra's algorithm on.
		start (hashable): The starting vertex for the search.

	Returns:
		tuple: A tuple containing a dictionary of visited nodes and their corresponding shortest distances from the start node, and a dictionary of the shortest paths.
	"""
	visited = {start: 0}  # Dictionary to store visited nodes and their distances from the start node
	path = {}  # Dictionary to store the shortest paths from the start node to other nodes

	# Set of all nodes in the graph
	nodes = set(graph.nodes)

	# While there are still unvisited nodes
	while nodes:
		min_node = None  # Initialize min_node to None

		# Find the node with the smallest distance from the start node
		for node in nodes:
			if node in visited:
				if min_node is None:
					min_node = node  # If min_node is not initialized, set it to the current node
				elif visited[node] < visited[min_node]:
					min_node = node  # If the current node has a smaller distance, set min_node to it

		# If all nodes are visited, break the loop
		if min_node is None:
			break

		# Remove the visited node from the set of unvisited nodes
		nodes.remove(min_node)

		# Get the current distance of the visited node from the start node
		current_weight = visited[min_node]

		# Update the distances and paths of the unvisited neighbors of the visited node
		for edge in graph.edges[min_node]:
			if min_node == edge:
				break  # Skip self-loops
			weight = current_weight + graph.weights[(min_node, edge)]
			if edge not in visited or weight < visited[edge]:
				visited[edge] = weight  # Update the distance if a shorter path is found
				path[edge] = min_node  # Update the path to the visited node

	return visited, path

# Prim's Algorithm
def prim(graph, start):
	"""Prim's algorithm for finding the minimum spanning tree of a graph.

	Args:
		graph (Graph): The graph to perform Prim's algorithm on.
		start (hashable): The starting vertex for the search.

	Returns:
		list: The list of nodes in the minimum spanning tree.
	"""
	visited = set()  # Set to keep track of visited nodes
	stack = [start]  # Stack to perform Prim's algorithm

	while stack:
		vertex = stack.pop()  # Pop a vertex from the stack

		# If the vertex is not visited, mark it as visited and add
		# its unvisited neighbors to the stack
		if vertex not in visited:
			visited.add(vertex)
			stack.extend(graph.nodes[vertex] - visited)

	return visited

# Transpose Graph
def transpose_graph(graph):
	"""Transposes a directed graph."""
	transpose = DiGraph()
	for node in graph.nodes:
		transpose.add_node(node)
	for node in graph.nodes:
		for adjacent in graph.nodes[node]:
			transpose.add_edge(adjacent, node)
	return transpose

# Strongly Connected Components
def strongly_connected_components(graph):
	"""Performs the Strongly Connected Components algorithm on a graph.

	Args:
	    graph (Graph): The graph to perform the algorithm on.

	Returns:
	    list: A list of lists, where each inner list represents a strongly
	          connected component in the graph.
	"""
	# Initialize variables
	currently_connected = []  # List of nodes in the current component
	to_visit = list(graph.nodes.keys())  # List of nodes to visit
	next = []  # Stack of nodes to visit next
	components = []  # List of strongly connected components

	# Add starting node to stack
	next.append(to_visit.pop())

	# Perform the algorithm
	while to_visit:
		# Perform Depth-First Search from each unvisited node
		while next:
			node = next.pop()
			currently_connected.append(node)
			for edge in graph.nodes[node]:
				# If the edge is not visited and a new path is found,
				# add it to the stack and remove it from to_visit
				if dfs_search(graph, edge, node) and not edge in currently_connected:
					next.append(edge)
					to_visit.remove(edge)

		# Add the current component to the list of components
		components.append(currently_connected)
		currently_connected = []

		# If there are still nodes to visit, add the next node to visit to the stack
		if to_visit:
			next.append(to_visit.pop())

	return components

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
	
	print("DFS starting with A")
	print(dfs(G, "A"))
	print("DFS starting with H")
	print(dfs(G, "H"))

	print("BFS starting with A")
	print(bfs(G, "A"))
	print("BFS starting with H")
	print(bfs(G, "H"))

	G = di_graph()
	#Do stuff here

	print("DFS starting with 1")
	print(dfs(G, "1"))
	print("Digraph SCCs")
	print(strongly_connected_components(G))

	G = weighted_graph()
	#Do stuff here
	
	print("Dijkstra starting with A")
	print(dijkstra(G, "A"))
	