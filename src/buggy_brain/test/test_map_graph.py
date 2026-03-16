import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'buggy_brain'))

from map_graph import find_shortest_path, get_node_coordinates, get_path_coordinates, EDGES, NODES


def test_main_gate_to_library():
    path = find_shortest_path(EDGES, 'MAIN_GATE', 'LIBRARY')
    assert path == ['MAIN_GATE', 'HUB', 'LIBRARY']


def test_main_gate_to_admin():
    path = find_shortest_path(EDGES, 'MAIN_GATE', 'ADMIN_BLOCK')
    assert path == ['MAIN_GATE', 'HUB', 'ADMIN_BLOCK']


def test_parking_to_canteen():
    path = find_shortest_path(EDGES, 'PARKING', 'CANTEEN')
    assert path == ['PARKING', 'MAIN_GATE', 'HUB', 'CANTEEN']


def test_same_start_and_goal():
    path = find_shortest_path(EDGES, 'HUB', 'HUB')
    assert path == ['HUB']


def test_invalid_node_returns_empty():
    path = find_shortest_path(EDGES, 'MAIN_GATE', 'NONEXISTENT')
    assert path == []


def test_all_nodes_reachable_from_main_gate():
    for destination in NODES:
        if destination == 'MAIN_GATE':
            continue
        path = find_shortest_path(EDGES, 'MAIN_GATE', destination)
        assert len(path) > 0, f"No path found to {destination}"


def test_get_node_coordinates():
    coords = get_node_coordinates('LIBRARY')
    assert coords == (40.0, 0.0)


def test_get_node_coordinates_invalid():
    coords = get_node_coordinates('NOWHERE')
    assert coords is None


def test_get_path_coordinates():
    path = ['MAIN_GATE', 'HUB', 'LIBRARY']
    coords = get_path_coordinates(path)
    assert coords == [(-40.0, 0.0), (0.0, 0.0), (40.0, 0.0)]


def test_path_is_reversible():
    path_forward = find_shortest_path(EDGES, 'LIBRARY', 'PARKING')
    path_reverse = find_shortest_path(EDGES, 'PARKING', 'LIBRARY')
    assert path_forward == list(reversed(path_reverse))
