#!/usr/bin/env python3
"""
map_graph.py  v3.0
──────────────────────────────────────────────────────────────
Campus road graph + Dijkstra for SRM Autonomous Buggy.

Real SRM Trichy campus — updated to match world v3.0:
  BUGGY_HUB  = (  0.0,   0.0)  ← Buggy home base / roundabout
  SRM_IST    = (  0.0,  50.0)  ← SRM Institute   (North, 50 m)
  SRM_HOSP   = ( 50.0,   0.0)  ← SRM Hospital    (East,  50 m)
  SRM_TEMPLE = (  0.0, -50.0)  ← SRM Temple      (South, 50 m)

Plan Reference: §2.2 ODD Map
"""
import heapq

# ── §2.2 ODD — Named campus destinations ─────────────────────
NODES = {
    'BUGGY_HUB':  ( 0.0,   0.0),
    'SRM_IST':    ( 0.0,  50.0),
    'SRM_HOSP':   (50.0,   0.0),
    'SRM_TEMPLE': ( 0.0, -50.0),
}

# ── §2.2 — Road network (metres) ─────────────────────────────
EDGES = {
    'BUGGY_HUB':  [('SRM_IST', 50.0), ('SRM_HOSP', 50.0), ('SRM_TEMPLE', 50.0)],
    'SRM_IST':    [('BUGGY_HUB', 50.0)],
    'SRM_HOSP':   [('BUGGY_HUB', 50.0)],
    'SRM_TEMPLE': [('BUGGY_HUB', 50.0)],
}

# ── Terminal menu shown to user ───────────────────────────────
DESTINATION_MENU = {
    'A': 'SRM_IST',
    'B': 'SRM_HOSP',
    'C': 'SRM_TEMPLE',
}

DESTINATION_DISPLAY = {
    'SRM_IST':    'SRM Institute of Science & Technology (North)',
    'SRM_HOSP':   'SRM Hospital / Medical College (East)',
    'SRM_TEMPLE': 'SRM Campus Temple (South)',
    'BUGGY_HUB':  'Buggy Hub — Home Base',
}


def find_shortest_path(graph: dict[str, list[tuple[str, float]]], start: str, goal: str) -> list[str]:
    """
    Dijkstra shortest-path — §5.2 Path Planning.
    Returns ordered list of node names: start → ... → goal
    """
    pq: list[tuple[float, str]] = [(0.0, start)]
    distances: dict[str, float] = {n: float('inf') for n in graph}
    distances[start] = 0.0
    previous: dict[str, str | None] = {n: None for n in graph}

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
    return list(reversed(path))


if __name__ == '__main__':
    print("=" * 60)
    print("  SRM Autonomous Buggy — map_graph.py v3.0 test")
    print("=" * 60)

    tests = [
        ('BUGGY_HUB', 'SRM_IST',    ['BUGGY_HUB', 'SRM_IST']),
        ('BUGGY_HUB', 'SRM_HOSP',   ['BUGGY_HUB', 'SRM_HOSP']),
        ('BUGGY_HUB', 'SRM_TEMPLE', ['BUGGY_HUB', 'SRM_TEMPLE']),
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
        print(f"     Dist : {EDGES[start][['SRM_IST','SRM_HOSP','SRM_TEMPLE'].index(goal)][1]} m")

    print()
    print("✅ All tests passed." if all_ok else "❌ Tests FAILED — fix before integrating.")
    print("=" * 60)