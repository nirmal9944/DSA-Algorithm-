import heapq
import math

class Node:
    def __init__(self, name, h_value=0):
        self.name = name
        self.g = float('inf')  # Cost from start to this node
        self.h = h_value       # Heuristic value to destination
        self.f = float('inf')  # Total cost (g + h)
        self.parent = None
        self.adjacent = {}     # Dictionary of neighbors with costs
        
    def __lt__(self, other):
        # For heapq priority queue comparison
        return self.f < other.f
    
    def __repr__(self):
        return self.name

class Graph:
    def __init__(self):
        self.nodes = {}
    
    def add_node(self, name, h_value=0):
        self.nodes[name] = Node(name, h_value)
        return self.nodes[name]
    
    def add_edge(self, node1, node2, cost):
        if node1 in self.nodes and node2 in self.nodes:
            self.nodes[node1].adjacent[node2] = cost
            self.nodes[node2].adjacent[node1] = cost  # Assuming undirected graph

def a_star_search(graph, start_name, destination_name):
    """
    A* Search Algorithm Implementation
    """
    # Get start and destination nodes
    start = graph.nodes[start_name]
    destination = graph.nodes[destination_name]
    
    # Initialize the start node
    start.g = 0
    start.f = start.g + start.h
    start.parent = None
    
    # Initialize open and closed lists
    open_list = []
    closed_list = set()
    
    # Add start node to open list
    heapq.heappush(open_list, start)
    
    # Track nodes in open list for faster lookup
    open_set = {start_name}
    
    print(f"Starting A* Search from {start_name} to {destination_name}")
    print("-" * 50)
    
    while open_list:
        # Get node with smallest f value from open list
        current = heapq.heappop(open_list)
        open_set.remove(current.name)
        
        print(f"Exploring node: {current.name} (g={current.g}, h={current.h}, f={current.f})")
        
        # If we reached the destination, reconstruct and return path
        if current.name == destination_name:
            print(f"\nDestination {destination_name} found!")
            return reconstruct_path(current)
        
        # Add current node to closed list
        closed_list.add(current.name)
        
        # Explore neighbors
        for neighbor_name, cost in current.adjacent.items():
            neighbor = graph.nodes[neighbor_name]
            
            # Skip if neighbor is in closed list
            if neighbor_name in closed_list:
                continue
            
            # Calculate tentative g score
            tentative_g = current.g + cost
            
            # If neighbor is not in open list, or we found a better path
            if neighbor_name not in open_set or tentative_g < neighbor.g:
                # Update parent and costs
                neighbor.parent = current
                neighbor.g = tentative_g
                neighbor.f = neighbor.g + neighbor.h
                
                # Add to open list if not already there
                if neighbor_name not in open_set:
                    heapq.heappush(open_list, neighbor)
                    open_set.add(neighbor_name)
                    print(f"  Added {neighbor_name} to open list (g={neighbor.g}, f={neighbor.f})")
                else:
                    # Update the priority in the heap
                    # We need to re-heapify by removing and re-adding
                    for i, node in enumerate(open_list):
                        if node.name == neighbor_name:
                            open_list[i] = neighbor
                            heapq.heapify(open_list)
                            print(f"  Updated {neighbor_name} in open list (g={neighbor.g}, f={neighbor.f})")
                            break
    
    print(f"\nNo path found from {start_name} to {destination_name}")
    return None

def reconstruct_path(node):
    """
    Reconstruct the path from destination back to start
    """
    path = []
    total_cost = node.g
    while node:
        path.append(node.name)
        node = node.parent
    
    path.reverse()
    return path, total_cost

def create_sample_graph():
    """
    Create a sample graph with heuristic values
    (You should modify this with your actual graph from the image)
    """
    graph = Graph()
    
    # Add nodes with heuristic values (straight-line distance to destination 'G')
    # These heuristic values are examples - use your actual values from the image
    graph.add_node('A', h_value=10)  # Heuristic to G
    graph.add_node('B', h_value=8)   # Heuristic to G
    graph.add_node('C', h_value=5)   # Heuristic to G
    graph.add_node('D', h_value=7)   # Heuristic to G
    graph.add_node('E', h_value=3)   # Heuristic to G
    graph.add_node('F', h_value=6)   # Heuristic to G
    graph.add_node('G', h_value=0)   # Destination heuristic is 0
    
    # Add edges with costs (modify with your actual graph from the image)
    graph.add_edge('A', 'B', 6)
    graph.add_edge('A', 'F', 3)
    graph.add_edge('B', 'C', 3)
    graph.add_edge('B', 'D', 2)
    graph.add_edge('C', 'D', 1)
    graph.add_edge('C', 'E', 5)
    graph.add_edge('D', 'E', 8)
    graph.add_edge('E', 'G', 5)
    graph.add_edge('F', 'G', 1)
    
    return graph

def main():
    # Create the graph (modify this with your actual graph from the image)
    graph = create_sample_graph()
    
    print("Available nodes:", list(graph.nodes.keys()))
    
    # Get start node from user
    while True:
        start_node = input("\nEnter the start node: ").strip().upper()
        if start_node in graph.nodes:
            break
        print(f"Error: Node '{start_node}' not found. Available nodes: {list(graph.nodes.keys())}")
    
    # Set destination (modify if different in your image)
    destination_node = 'G'  # Change this if your destination is different
    
    print(f"\nDestination node is set to: {destination_node}")
    
    # Run A* search
    result = a_star_search(graph, start_node, destination_node)
    
    if result:
        path, total_cost = result
        print(f"\nOptimal path found!")
        print(f"Path: {' -> '.join(path)}")
        print(f"Total cost: {total_cost}")
    
    # Optional: Display the graph structure
    print("\n" + "=" * 50)
    print("Graph Structure:")
    print("=" * 50)
    for node_name, node in graph.nodes.items():
        print(f"{node_name} (h={node.h}): {node.adjacent}")

if __name__ == "__main__":
    main()
