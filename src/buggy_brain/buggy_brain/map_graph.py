import heapq

# Campus road network — node names map to (x, y) coordinates in metres
# Origin (0, 0) = centre of campus
NODES = {
    'MAIN_GATE':   (-40.0,   0.0),
    'HUB':         (  0.0,   0.0),
    'LIBRARY':     ( 40.0,   0.0),
    'ADMIN_BLOCK': (  0.0,  30.0),
    'CANTEEN':     (  0.0, -30.0),
    'PARKING':     (-40.0,  30.0),
}

# Adjacency list — (neighbour, distance_in_metres)
# All edges are bidirectional so every pair is listed twice
EDGES = {
    'MAIN_GATE':   [('HUB', 40.0),     ('PARKING', 30.0)],
    'HUB':         [('MAIN_GATE', 40.0), ('LIBRARY', 40.0),
                    ('ADMIN_BLOCK', 30.0), ('CANTEEN', 30.0)],
    'LIBRARY':     [('HUB', 40.0)],
    'ADMIN_BLOCK': [('HUB', 30.0),     ('PARKING', 50.0)],
    'CANTEEN':     [('HUB', 30.0)],
    'PARKING':     [('MAIN_GATE', 30.0), ('ADMIN_BLOCK', 50.0)],
}


def find_shortest_path(graph, start, goal):
    """
    Dijkstra's algorithm — returns ordered list of node names.
    Returns empty list if no path exists.
    """
    if start not in graph or goal not in graph:
        return []

    if start == goal:
        return [start]

    pq = [(0.0, start)]
    distances = {n: float('inf') for n in graph}
    distances[start] = 0.0
    previous = {n: None for n in graph}
    visited = set()

    while pq:
        cost, node = heapq.heappop(pq)

        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            break

        for neighbour, weight in graph[node]:
            new_cost = cost + weight
            if new_cost < distances[neighbour]:
                distances[neighbour] = new_cost
                previous[neighbour] = node
                heapq.heappush(pq, (new_cost, neighbour))

    # Reconstruct path
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = previous[node]

    path.reverse()

    # If path doesn't start at 'start', no route was found
    if path[0] != start:
        return []

    return path


def get_node_coordinates(node_name):
    """
    Returns (x, y) tuple for a given node name.
    Used by path_planner_node to build ROS PoseStamped messages.
    """
    return NODES.get(node_name, None)


def get_path_coordinates(path):
    """
    Converts a list of node names into a list of (x, y) tuples.
    Used by waypoint_follower to get actual drive targets.
    """
    return [NODES[n] for n in path if n in NODES]


if __name__ == '__main__':
    print("=" * 50)
    print("SRM Buggy — map_graph.py standalone test")
    print("=" * 50)

    tests = [
        ('MAIN_GATE', 'LIBRARY'),
        ('MAIN_GATE', 'ADMIN_BLOCK'),
        ('MAIN_GATE', 'CANTEEN'),
        ('PARKING',   'CANTEEN'),
        ('LIBRARY',   'PARKING'),
        ('HUB',       'HUB'),       # same start and goal
    ]

    all_passed = True
    for start, goal in tests:
        path = find_shortest_path(EDGES, start, goal)
        coords = get_path_coordinates(path)
        print(f"\n{start} -> {goal}")
        print(f"  Path  : {' -> '.join(path)}")
        print(f"  Coords: {coords}")
        if not path:
            print("  FAIL — no path found")
            all_passed = False

    print("\n" + "=" * 50)
    if all_passed:
        print("ALL TESTS PASSED — ready for ROS 2 integration")
    else:
        print("SOME TESTS FAILED — check graph connectivity")
    print("=" * 50)