import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from classify_multifloor_rooms_qwen import group_stairwells, overlay_transition_roles


def test_transition_roles_override_geometric_corridor_guess():
    graph = {"rooms": [
        {"id": 10, "merged_from_region_ids": [10], "space_role": "room", "semantic_type": "unknown", "polygon_xy_m": [[0, 0], [1, 0], [1, 1], [0, 1]]},
        {"id": 9, "merged_from_region_ids": [9], "space_role": "transition_space", "semantic_type": "corridor", "polygon_xy_m": [[3, 0], [4, 0], [4, 1], [3, 1]]},
    ]}
    aware = {"transition_spaces": [{"id": "transition_2", "source_region_ids": [10], "connected_floors": [2, 3], "confidence": .9}]}
    links = overlay_transition_roles(graph, aware, 2)
    assert graph["rooms"][0]["semantic_type"] == "stairwell"
    assert graph["rooms"][1]["space_role"] == "room"
    assert links[0]["room_id"] == 10


def test_touching_consecutive_transitions_form_one_stairwell():
    links = [
        {"transition_id": "transition_1", "floor": 2, "room_id": 11, "polygon_xy_m": [[0, 0], [1, 0], [1, 1], [0, 1]], "connected_floors": [1, 2], "confidence": .8},
        {"transition_id": "transition_2", "floor": 2, "room_id": 10, "polygon_xy_m": [[1, 0], [2, 0], [2, 1], [1, 1]], "connected_floors": [2, 3], "confidence": .9},
        {"transition_id": "transition_1", "floor": 1, "room_id": 5, "polygon_xy_m": [[0, 0], [1, 0], [1, 1], [0, 1]], "connected_floors": [1, 2], "confidence": .8},
        {"transition_id": "transition_2", "floor": 3, "room_id": 12, "polygon_xy_m": [[1, 0], [2, 0], [2, 1], [1, 1]], "connected_floors": [2, 3], "confidence": .9},
    ]
    result = group_stairwells(links, .3)
    assert len(result) == 1
    assert result[0]["floors"] == [1, 2, 3]
    assert result[0]["floor_region_ids"] == {"1": [5], "2": [10, 11], "3": [12]}
