import heapq

# Section 2.2 ODD Table - Campus road network topology
NODES = {
    'START': (-20.0, 0.0),
    'HUB':   (0.0, 0.0),
    'A':     (-20.0, 0.0),   # Main Gate (same as start for demo)
    'B':     (20.0, 0.0),    # Library Block
    'C':     (0.0, 20.0),    # Admin Block
}

EDGES = {
    'START': [('HUB', 20.0)],
    'HUB':   [('START', 20.0), ('B', 20.0), ('C', 20.0)],
    'B':     [('HUB', 20.0)],
    'C':     [('HUB', 20.0)],
}


def find_shortest_path(graph, start, goal):
    """
    Computes the shortest path using Dijkstra's algorithm.
    Required for Day 3 Team Bravo delivery.
    """
    pq = [(0, start)]
    distances = {n: float('inf') for n in graph}
    distances[start] = 0
    previous = {n: None for n in graph}
    
    while pq:
        cost, node = heapq.heappop(pq)
        
        if node == goal:
            break
            
        for neighbor, weight in graph[node]:
            new_cost = cost + weight
            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                previous[neighbor] = node
                heapq.heappush(pq, (new_cost, neighbor))
                
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = previous[node]
        
    return list(reversed(path))


if __name__ == '__main__':
    # Day 3 Critical Testing Step
    print("Testing map_graph.py standalone Dijkstra module:")
    
    route_b = find_shortest_path(EDGES, 'START', 'B')
    print(f"START -> B: {route_b}")
    assert route_b == ['START', 'HUB', 'B'], "Failed START->B path"
    
    route_c = find_shortest_path(EDGES, 'START', 'C')
    print(f"START -> C: {route_c}")
    assert route_c == ['START', 'HUB', 'C'], "Failed START->C path"
    
    print("\n✅ All routing tests passed. Ready for ROS 2 integration.")
