import heapq

class Node:
    def __init__(self, name, h=0):
        self.name = name
        self.h = h
        self.g = float('inf')
        self.f = float('inf')
        self.parent = None
        self.neighbors = {}
    
    def __lt__(self, other):
        return self.f < other.f

class Graph:
    def __init__(self):
        self.nodes = {}
    
    def add_node(self, name, h=0):
        self.nodes[name] = Node(name, h)
    
    def add_edge(self, a, b, cost):
        if a in self.nodes and b in self.nodes:
            self.nodes[a].neighbors[b] = cost
            self.nodes[b].neighbors[a] = cost

def a_star(graph, start, goal):
    start_node = graph.nodes[start]
    goal_node = graph.nodes[goal]
    
    start_node.g = 0
    start_node.f = start_node.g + start_node.h
    start_node.parent = None
    
    open_list = []
    open_set = set()
    closed_set = set()
    
    heapq.heappush(open_list, start_node)
    open_set.add(start)
    
    while open_list:
        current = heapq.heappop(open_list)
        open_set.remove(current.name)
        
        if current.name == goal:
            return reconstruct(current)
        
        closed_set.add(current.name)
        
        for neighbor_name, cost in current.neighbors.items():
            if neighbor_name in closed_set:
                continue
            
            neighbor = graph.nodes[neighbor_name]
            new_g = current.g + cost
            
            if neighbor_name not in open_set or new_g < neighbor.g:
                neighbor.parent = current
                neighbor.g = new_g
                neighbor.f = neighbor.g + neighbor.h
                
                if neighbor_name not in open_set:
                    heapq.heappush(open_list, neighbor)
                    open_set.add(neighbor_name)
                else:
                    for i, node in enumerate(open_list):
                        if node.name == neighbor_name:
                            open_list[i] = neighbor
                            heapq.heapify(open_list)
                            break
    
    return None, None

def reconstruct(node):
    path = []
    total_cost = node.g
    while node:
        path.append(node.name)
        node = node.parent
    return path[::-1], total_cost

def main():
    g = Graph()
    
    g.add_node('A', 10)
    g.add_node('B', 8)
    g.add_node('C', 5)
    g.add_node('D', 7)
    g.add_node('E', 3)
    g.add_node('F', 6)
    g.add_node('G', 0)
    
    g.add_edge('A', 'B', 6)
    g.add_edge('A', 'F', 3)
    g.add_edge('B', 'C', 3)
    g.add_edge('B', 'D', 2)
    g.add_edge('C', 'D', 1)
    g.add_edge('C', 'E', 5)
    g.add_edge('D', 'E', 8)
    g.add_edge('E', 'G', 5)
    g.add_edge('F', 'G', 1)
    
    start = input("Enter start node: ").upper()
    
    if start not in g.nodes:
        print("Invalid node")
        return
    
    path, cost = a_star(g, start, 'G')
    
    if path:
        print(f"Path: {' -> '.join(path)}")
        print(f"Cost: {cost}")
    else:
        print("No path found")

if __name__ == "__main__":
    main()
