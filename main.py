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
		# Set of vertices
		self.nodes = {}

	def add_node(self, value):
		self.nodes[value] = set()

	def add_edge(self, from_node, to_node):
		self.nodes[from_node].add(to_node)
		self.nodes[to_node].add(from_node)

# DiGraph Class
class DiGraph:
	def __init__(self):
		self.nodes = set()
		self.edges = collections.defaultdict(list)

	def add_node(self, value):
		self.nodes.add(value)

	def add_edge(self, from_node, to_node):
		self.edges[from_node].append(to_node)

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