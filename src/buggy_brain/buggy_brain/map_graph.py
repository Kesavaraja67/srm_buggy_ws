#!/usr/bin/env python3
"""
map_graph.py  v4.0
──────────────────────────────────────────────────────────────
Campus road graph + Dijkstra for SRM Autonomous Buggy.

Coordinates matched to srm_campus.world:
  BUGGY_HUB  = (-13.0,  0.0)   Parking Bay (West)
  SRM_IST    = (  0.0, 50.0)   North building
  SRM_HOSP   = ( 50.0,  0.0)   East building
  SRM_TEMPLE = (  0.0,-50.0)   South building

Roundabout at (0,0) with radius ~7m road centerline.
Intermediate nodes (NW/NE/SW/SE) ensure smooth turns.
"""
import heapq
from typing import Dict, List, Tuple, Optional

# ── Named campus destinations ────────────────────────────────
NODES = {
    'BUGGY_HUB':  (-13.0,   0.0),   # Parking Bay
    'SRM_IST':    (  0.0,  50.0),   # North building
    'SRM_HOSP':   ( 50.0,   0.0),   # East building
    'SRM_TEMPLE': (  0.0, -50.0),   # South building
    'RND_N':      (  0.0,   7.0),   # Roundabout North
    'RND_S':      (  0.0,  -7.0),   # Roundabout South
    'RND_E':      (  7.0,   0.0),   # Roundabout East
    'RND_W':      ( -7.0,   0.0),   # Roundabout West
    'RND_NW':     ( -5.0,   5.0),   # Roundabout NW
    'RND_NE':     (  5.0,   5.0),   # Roundabout NE
    'RND_SW':     ( -5.0,  -5.0),   # Roundabout SW
    'RND_SE':     (  5.0,  -5.0),   # Roundabout SE
}

# ── Road network (bidirectional, metres) ─────────────────────
EDGES = {
    'BUGGY_HUB':  [('RND_W', 6.0)],
    'SRM_IST':    [('RND_N', 43.0)],
    'SRM_HOSP':   [('RND_E', 43.0)],
    'SRM_TEMPLE': [('RND_S', 43.0)],

    'RND_N':      [('SRM_IST', 43.0),  ('RND_NW', 5.5), ('RND_NE', 5.5)],
    'RND_E':      [('SRM_HOSP', 43.0), ('RND_NE', 5.5), ('RND_SE', 5.5)],
    'RND_S':      [('SRM_TEMPLE', 43.0), ('RND_SE', 5.5), ('RND_SW', 5.5)],
    'RND_W':      [('BUGGY_HUB', 6.0), ('RND_NW', 5.5), ('RND_SW', 5.5)],

    # Intermediate curve nodes (bidirectional)
    'RND_NW':     [('RND_N', 5.5), ('RND_W', 5.5)],
    'RND_NE':     [('RND_N', 5.5), ('RND_E', 5.5)],
    'RND_SE':     [('RND_S', 5.5), ('RND_E', 5.5)],
    'RND_SW':     [('RND_S', 5.5), ('RND_W', 5.5)],
}

# ── Terminal menu mapping ─────────────────────────────────────
VALID_DESTINATIONS = {
    'A': 'SRM_IST',
    'B': 'SRM_HOSP',
    'C': 'SRM_TEMPLE',
    'D': 'BUGGY_HUB',
}

DESTINATION_DISPLAY = {
    'SRM_IST':    'SRM Institute of Science & Technology (North)',
    'SRM_HOSP':   'SRM Hospital / Medical College (East)',
    'SRM_TEMPLE': 'SRM Campus Temple (South)',
    'BUGGY_HUB':  'Buggy Hub — Home Base',
}

# ── Shared multi-line menu UI ────────────────────────────────
MENU = (
    "\n"
    "========================================\n"
    "  SRM Autonomous Buggy — Command Center\n"
    "========================================\n"
    "  [A] SRM Institute (North)\n"
    "  [B] SRM Hospital (East)\n"
    "  [C] SRM Temple (South)\n"
    "  [D] Buggy Hub (Home)\n"
    "  [Q] Emergency Shutdown\n"
    "========================================\n"
    "Select Destination: "
)


def find_shortest_path(graph: Dict[str, List[Tuple[str, float]]], start: str, goal: str) -> List[str]:
    """
    Dijkstra shortest-path.
    Returns ordered list of node names: start -> ... -> goal
    """
    if start not in graph or goal not in graph:
        return []

    pq: List[Tuple[float, str]] = [(0.0, start)]
    distances: Dict[str, float] = {n: float('inf') for n in graph}
    distances[start] = 0.0
    previous: Dict[str, Optional[str]] = {n: None for n in graph}

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

    path, node = [], goal
    while node is not None:
        path.append(node)
        node = previous[node]
    if len(path) == 1 and path[0] != start:
        return []
    return list(reversed(path))


def get_path_coordinates(path):
    return [NODES[node] for node in path if node in NODES]


if __name__ == '__main__':
    print("=" * 60)
    print("  SRM Autonomous Buggy — map_graph.py v4.0 test")
    print("=" * 60)

    tests = [
        ('BUGGY_HUB', 'SRM_IST',    ['BUGGY_HUB', 'RND_W', 'RND_NW', 'RND_N', 'SRM_IST']),
        ('BUGGY_HUB', 'SRM_HOSP',   ['BUGGY_HUB', 'RND_W', 'RND_NW', 'RND_N', 'RND_NE', 'RND_E', 'SRM_HOSP']),
        ('BUGGY_HUB', 'SRM_TEMPLE', ['BUGGY_HUB', 'RND_W', 'RND_SW', 'RND_S', 'SRM_TEMPLE']),
        ('SRM_IST',    'BUGGY_HUB', ['SRM_IST', 'RND_N', 'RND_NW', 'RND_W', 'BUGGY_HUB']),
        ('SRM_HOSP',   'BUGGY_HUB', ['SRM_HOSP', 'RND_E', 'RND_NE', 'RND_N', 'RND_NW', 'RND_W', 'BUGGY_HUB']),
        ('SRM_TEMPLE', 'BUGGY_HUB', ['SRM_TEMPLE', 'RND_S', 'RND_SW', 'RND_W', 'BUGGY_HUB']),
    ]

    all_ok = True
    for start, goal, expected in tests:
        result = find_shortest_path(EDGES, start, goal)
        ok = (result == expected)
        if not ok:
            all_ok = False
        print(f"\n  {'✅' if ok else '❌'}  {start} → {goal}")
        print(f"     {DESTINATION_DISPLAY[goal]}")
        print(f"     Path : {' → '.join(result)}")
        if len(result) > 1:
            total_dist = sum(
                next(w for neighbor, w in EDGES[result[i]] if neighbor == result[i + 1])
                for i in range(len(result) - 1)
            )
            print(f"     Dist : {total_dist} m")
        else:
            print("     Dist : 0.0 m")

    print()
    print("✅ All tests passed." if all_ok else "❌ Tests FAILED — fix before integrating.")
    print("=" * 60)
