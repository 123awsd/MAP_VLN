"""Build reproducible, map-grounded long-instruction benchmark episodes."""

from __future__ import annotations

import math
import random
from collections import Counter
from typing import Any

from .candidate_poses import generate_candidates
from .grid_map import OccupancyGrid
from .task_graph import normalize_and_validate_task_graph


TASK_LABELS = [
    "bed", "sofa", "television", "table", "dining table", "coffee table",
    "desk", "chair", "armchair", "cabinet", "dresser", "nightstand",
    "bookshelf", "shelf", "refrigerator", "oven", "microwave", "sink",
    "toilet", "bathtub", "shower", "mirror", "lamp", "floor lamp",
    "plant", "vase", "picture", "painting", "clock", "stool", "bench",
    "wardrobe", "countertop", "kitchen island", "washing machine",
    "speaker", "photo frame", "newspaper", "cutting board", "camera", "iron",
    "ottoman", "piano", "fireplace", "vanity", "stove", "faucet",
]

MOVABLE_LABELS = {
    "chair", "armchair", "stool", "bench", "plant", "vase", "lamp",
    "speaker",
    "newspaper", "cutting board", "camera", "iron", "ottoman",
}

ZH = {
    "bed": "床", "sofa": "沙发", "television": "电视", "table": "桌子",
    "dining table": "餐桌", "coffee table": "茶几", "desk": "书桌",
    "chair": "椅子", "armchair": "扶手椅", "cabinet": "柜子",
    "dresser": "斗柜", "nightstand": "床头柜", "bookshelf": "书架",
    "shelf": "置物架", "refrigerator": "冰箱", "oven": "烤箱",
    "microwave": "微波炉", "sink": "水槽", "toilet": "马桶",
    "bathtub": "浴缸", "shower": "淋浴区", "mirror": "镜子",
    "lamp": "灯", "floor lamp": "落地灯", "plant": "绿植", "vase": "花瓶",
    "picture": "挂画", "painting": "画", "clock": "时钟", "stool": "凳子",
    "bench": "长凳", "wardrobe": "衣柜", "countertop": "操作台",
    "kitchen island": "厨房岛台", "washing machine": "洗衣机",
    "speaker": "音箱", "photo frame": "相框", "newspaper": "报纸",
    "cutting board": "砧板", "camera": "相机", "iron": "熨斗",
    "ottoman": "脚凳", "piano": "钢琴", "fireplace": "壁炉",
    "vanity": "梳妆台", "stove": "炉灶", "faucet": "水龙头",
}

EXCLUDED = {
    "wall", "floor", "ceiling", "door", "window", "column", "stairs",
    "railing", "room divider", "curtain", "blinds", "rug", "carpet",
}

RELATION_ZH = {
    None: "到{name}附近确认一下", "near": "到{name}附近确认一下",
    "front": "到{name}正面查看", "left": "从{name}左侧观察",
    "right": "从{name}右侧观察", "behind": "从{name}后侧检查",
}

FALLBACK_HINTS = {
    "chair": {"table", "dining table", "desk", "sofa", "piano", "fireplace", "dresser"},
    "armchair": {"sofa", "coffee table", "table", "fireplace", "piano", "lamp"},
    "stool": {"countertop", "kitchen island", "table", "desk", "sofa"},
    "bench": {"table", "piano", "fireplace", "dresser", "sofa"},
    "ottoman": {"sofa", "armchair", "bed", "coffee table", "desk"},
    "newspaper": {"desk", "table", "coffee table", "countertop", "sofa", "cabinet"},
    "cutting board": {"countertop", "kitchen island", "sink", "stove", "cabinet"},
    "camera": {"desk", "table", "shelf", "cabinet", "sofa"},
    "iron": {"cabinet", "shelf", "table", "washing machine", "wardrobe"},
    "speaker": {"desk", "table", "television", "cabinet", "shelf"},
    "plant": {"window", "table", "cabinet", "shelf", "sofa"},
    "vase": {"table", "cabinet", "shelf", "countertop"},
    "lamp": {"nightstand", "desk", "table", "sofa", "bed"},
}


def _task(task_id: str, label: str, relation: str | None, prerequisites: list[str],
          *, active: bool = True, recovery: bool = False,
          verification_label: str | None = None) -> dict[str, Any]:
    return {
        "id": task_id,
        "action": "inspect",
        "target": {"label": label, "room": None, "reference": None},
        "verification_label": verification_label or label,
        "spatial_constraints": {
            "relation": relation,
            "distance_m": [0.8, 2.2],
            "height_m": None,
            "face_target": True,
            "visibility_required": True,
        },
        "prerequisites": prerequisites,
        "active_initially": active,
        "success_outcome": "found",
        "search_policy": {
            "mode": "semantic_recovery" if recovery else "fixed",
            "maximum_location_hypotheses": 3,
        },
    }


def _objects(scene_graph: dict[str, Any], allowed_object_ids: set[str] | None = None,
             ) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    preferred = {value for value in TASK_LABELS}
    values = []
    for room in scene_graph.get("rooms", []):
        if room.get("space_role", "room") != "room":
            continue
        for obj in room.get("objects", []):
            if allowed_object_ids is not None and str(obj["id"]) not in allowed_object_ids:
                continue
            label = str(obj.get("label", "")).strip().lower()
            if label in EXCLUDED:
                continue
            if label in preferred:
                values.append((TASK_LABELS.index(label), room, obj))
    values.sort(key=lambda item: (item[0], -float(item[2].get("probability", 0.0))))
    return [(room, obj) for _, room, obj in values]


def _select_anchors(grid: OccupancyGrid, scene_graph: dict[str, Any], count: int,
                    rng: random.Random, relations: list[str | None],
                    require_repeated_label: bool = False,
                    allowed_labels: set[str] | None = None,
                    excluded_labels: set[str] | None = None,
                    excluded_object_ids: set[str] | None = None,
                    allowed_object_ids: set[str] | None = None) -> list[dict[str, Any]]:
    pool = _objects(scene_graph, allowed_object_ids)
    label_counts = Counter(str(obj["label"]).lower() for _, obj in pool)
    feasible = []
    for room, obj in pool:
        label = str(obj["label"]).lower()
        if allowed_labels is not None and label not in allowed_labels:
            continue
        if excluded_labels and label in excluded_labels:
            continue
        if excluded_object_ids and str(obj["id"]) in excluded_object_ids:
            continue
        if require_repeated_label and label_counts[label] < 2:
            continue
        # Test every requested relation because side-specific viewpoints may be blocked.
        by_relation = {}
        for relation in set(relations) | {"near"}:
            probe = _task("probe", label, relation, [])
            candidates = generate_candidates(
                grid, scene_graph, probe, max_candidates=6,
                allowed_object_ids={str(obj["id"])}, prefer_room=False,
            )
            candidates = [
                item for item in candidates
                if grid.astar([0.0, 0.0], [item["pose"]["x"], item["pose"]["y"]]) is not None
            ]
            if candidates:
                by_relation[relation] = candidates
        feasible.append({"room": room, "object": obj, "by_relation": by_relation})

    selected: list[dict[str, Any]] = []
    used_ids: set[str] = set()
    used_labels: set[str] = set()
    for relation in relations:
        actual_relation = relation
        options = [
            item for item in feasible
            if relation in item["by_relation"] and str(item["object"]["id"]) not in used_ids
        ]
        if not options and relation != "near":
            actual_relation = "near"
            options = [
                item for item in feasible
                if "near" in item["by_relation"] and str(item["object"]["id"]) not in used_ids
            ]
        if not options:
            raise ValueError(f"scene has no feasible object for relation {relation}")

        def score(item: dict[str, Any]) -> tuple[float, float, float]:
            obj = item["object"]
            center = [float(value) for value in obj["center_xyz_m"][:2]]
            separation = min(
                (math.dist(center, old["object"]["center_xyz_m"][:2]) for old in selected),
                default=0.0,
            )
            label_bonus = 8.0 if str(obj["label"]).lower() not in used_labels else 0.0
            jitter = rng.random() * 0.05
            return label_bonus + separation + jitter, separation, float(obj.get("probability", 0.0))

        chosen = max(options, key=score)
        chosen = dict(chosen)
        chosen["relation"] = actual_relation
        selected.append(chosen)
        used_ids.add(str(chosen["object"]["id"]))
        used_labels.add(str(chosen["object"]["label"]).lower())
        if len(selected) == count:
            break
    return selected


def _natural_order(labels: list[str], relations: list[str | None], variant: int) -> str:
    names = [ZH.get(label, label) for label in labels]
    openings = [
        "出门前帮我做一次屋内巡检，把几处情况记录下来。",
        "趁现在有空，陪我按顺序巡视一下屋子。",
        "家里一会儿要来客人，麻烦按这个顺序检查并留一份影像记录。",
        "准备收拾屋子了，先帮我按顺序快速看一圈。",
        "做今天的例行检查时，请按下面的顺序把几处状态确认好。",
        "晚上休息前，麻烦按顺序巡查一下这些位置。",
        "我想更新一下屋内记录，请依次查看这几处。",
        "开始整理家具前，先按顺序帮我检查几个位置。",
        "今天的房屋巡检还差几项，请接着按顺序完成。",
        "为了确认屋内布置没有异常，请按顺序走一遍。",
    ]
    clauses = []
    for index, (name, relation) in enumerate(zip(names, relations)):
        lead = "先" if index == 0 else ("最后" if index == len(names) - 1 else ("接着" if index == 1 else "之后"))
        clauses.append(f"{lead}{RELATION_ZH[relation].format(name=name)}")
    return openings[variant % len(openings)] + "，".join(clauses) + "，确认没有异常就可以了。"


def build_scene_episodes(scene_id: str, scene_graph: dict[str, Any], grid: OccupancyGrid,
                         scene_index: int, seed: int = 20260825,
                         allowed_object_ids: set[str] | None = None) -> list[dict[str, Any]]:
    """Build ten deterministic episodes: 4 order, 3 conditional, 3 recovery."""
    rng = random.Random(seed + scene_index * 1009)
    episodes = []
    relations_by_episode = [
        ["front", "near", "left", "right"],
        ["near", "right", "front", "near"],
        ["left", "near", "front", "right"],
        ["right", "front", "near", "left"],
    ]

    for local_index in range(4):
        relations = relations_by_episode[local_index]
        anchors = _select_anchors(
            grid, scene_graph, 4, rng, relations, allowed_object_ids=allowed_object_ids,
        )
        actual_relations = [item["relation"] for item in anchors]
        tasks = []
        for index, anchor in enumerate(anchors):
            label = str(anchor["object"]["label"]).lower()
            tasks.append(_task(
                f"goal_{index + 1}", label, actual_relations[index],
                [] if index == 0 else [f"goal_{index}"],
            ))
        graph = normalize_and_validate_task_graph({
            "instruction": _natural_order(
                [str(item["object"]["label"]).lower() for item in anchors], actual_relations,
                scene_index + local_index,
            ),
            "tasks": tasks,
            "conditional_rules": [],
        })
        episodes.append(_package(scene_id, scene_index, local_index, "ordered_spatial", graph, anchors))

    for branch_index in range(3):
        relations = ["front", "near", "left", "right", "near"]
        try:
            source = _select_anchors(
                grid, scene_graph, 1, rng, [relations[1]], allowed_labels=MOVABLE_LABELS,
                allowed_object_ids=allowed_object_ids,
            )[0]
        except ValueError:
            source = _select_anchors(
                grid, scene_graph, 1, rng, [relations[1]],
                allowed_object_ids=allowed_object_ids,
            )[0]
        source_label = str(source["object"]["label"]).lower()
        try:
            others = _select_anchors(
                grid, scene_graph, 3, rng,
                [relations[0], relations[2], relations[3]],
                excluded_labels={source_label},
                excluded_object_ids={str(source["object"]["id"])},
                allowed_object_ids=allowed_object_ids,
            )
        except ValueError:
            others = _select_anchors(
                grid, scene_graph, 3, rng,
                [relations[0], relations[2], relations[3]],
                excluded_object_ids={str(source["object"]["id"])},
                allowed_object_ids=allowed_object_ids,
            )
        normal = [others[0], source, others[1], others[2]]
        normal_ids = {str(item["object"]["id"]) for item in normal}
        preferred_fallbacks = FALLBACK_HINTS.get(source_label, set())
        try:
            fallback = _select_anchors(
                grid, scene_graph, 1, rng, [relations[4]],
                allowed_labels=preferred_fallbacks or None,
                excluded_labels={source_label}, excluded_object_ids=normal_ids,
                allowed_object_ids=allowed_object_ids,
            )[0]
        except ValueError:
            fallback = _select_anchors(
                grid, scene_graph, 1, rng, [relations[4]],
                excluded_object_ids=normal_ids, allowed_object_ids=allowed_object_ids,
            )[0]
        anchors = normal + [fallback]
        actual_relations = [item["relation"] for item in anchors]
        tasks = []
        for index, anchor in enumerate(normal):
            tasks.append(_task(
                f"goal_{index + 1}", str(anchor["object"]["label"]).lower(), actual_relations[index],
                [] if index == 0 else [f"goal_{index}"],
            ))
        source_id = "goal_2"
        tasks.append(_task(
            "fallback_1", str(fallback["object"]["label"]).lower(), actual_relations[4],
            [source_id], active=False,
            verification_label=str(normal[1]["object"]["label"]).lower(),
        ))
        source_name = ZH.get(str(normal[1]["object"]["label"]).lower(), str(normal[1]["object"]["label"]))
        fallback_name = ZH.get(str(fallback["object"]["label"]).lower(), str(fallback["object"]["label"]))
        instruction = _natural_order(
            [str(item["object"]["label"]).lower() for item in normal], actual_relations[:4],
            scene_index + branch_index + 4,
        )
        instruction = instruction.removesuffix("，确认没有异常就可以了。")
        instruction += f"；如果在记录的位置没看到{source_name}，我记得它也可能在{fallback_name}附近，去那里找找。"
        graph = normalize_and_validate_task_graph({
            "instruction": instruction,
            "tasks": tasks,
            "conditional_rules": [{
                "source_task_id": source_id,
                "if_outcome": "not_found",
                "activate_task_ids": ["fallback_1"],
                "skip_task_ids": [],
            }],
        })
        branch_taken = ((scene_index * 3 + branch_index) % 2 == 0)
        episode = _package(
            scene_id, scene_index, 4 + branch_index, "ordered_conditional", graph,
            anchors, expected_branch="taken" if branch_taken else "skipped",
        )
        source_object_id = str(normal[1]["object"]["id"])
        if branch_taken:
            episode["controls"]["stale_object_ids"] = [source_object_id]
            episode["controls"]["found_object_ids"] = [str(fallback["object"]["id"])]
        else:
            episode["controls"]["found_object_ids"] = [source_object_id]
        episodes.append(episode)

    # Two recoverable stale targets and one deliberately exhausted target.
    for recovery_index in range(3):
        relations = ["front", "near", "left", "right"]
        try:
            repeated = _select_anchors(
                grid, scene_graph, 1, rng, [relations[1]], require_repeated_label=True,
                allowed_labels=MOVABLE_LABELS, allowed_object_ids=allowed_object_ids,
            )[0]
        except ValueError:
            repeated = _select_anchors(
                grid, scene_graph, 1, rng, [relations[1]], require_repeated_label=True,
                allowed_object_ids=allowed_object_ids,
            )[0]
        repeated_label = str(repeated["object"]["label"]).lower()
        try:
            remaining = _select_anchors(
                grid, scene_graph, 3, rng, [relations[0], relations[2], relations[3]],
                excluded_labels={repeated_label},
                excluded_object_ids={str(repeated["object"]["id"])},
                allowed_object_ids=allowed_object_ids,
            )
        except ValueError:
            remaining = _select_anchors(
                grid, scene_graph, 3, rng, [relations[0], relations[2], relations[3]],
                excluded_object_ids={str(repeated["object"]["id"])},
                allowed_object_ids=allowed_object_ids,
            )
        anchors = [remaining[0], repeated, remaining[1], remaining[2]]
        actual_relations = [item["relation"] for item in anchors]
        tasks = []
        for index, anchor in enumerate(anchors):
            tasks.append(_task(
                f"goal_{index + 1}", str(anchor["object"]["label"]).lower(), actual_relations[index],
                [] if index == 0 else [f"goal_{index}"], recovery=(index == 1),
            ))
        lost_label = str(repeated["object"]["label"]).lower()
        instruction = _natural_order(
            [str(item["object"]["label"]).lower() for item in anchors], actual_relations,
            scene_index + recovery_index + 7,
        )
        lost_name = ZH.get(lost_label, lost_label)
        instruction = instruction.removesuffix("，确认没有异常就可以了。")
        instruction += (
            f"。其中{lost_name}的历史位置可能已经不准确；如果记录的位置没有，"
            "就根据屋里的布置继续找，找到或确认相关区域已经搜完后再完成后面的检查。"
        )
        graph = normalize_and_validate_task_graph({
            "instruction": instruction,
            "tasks": tasks,
            "conditional_rules": [],
        })
        expected = "rediscovered" if recovery_index < 2 else "exhausted"
        episode = _package(
            scene_id, scene_index, 7 + recovery_index, "ordered_recovery", graph,
            anchors, expected_recovery=expected,
        )
        same_label_ids = sorted({
            str(obj["id"]) for _, obj in _objects(scene_graph, allowed_object_ids)
            if str(obj["label"]).lower() == lost_label
        })
        expected_id = str(repeated["object"]["id"])
        if expected == "rediscovered":
            alternatives = [value for value in same_label_ids if value != expected_id]
            if not alternatives:
                raise ValueError(f"no alternative {lost_label} for recovery episode")
            episode["controls"]["stale_object_ids"] = [expected_id]
            episode["controls"]["found_object_ids"] = alternatives
        else:
            episode["controls"]["stale_object_ids"] = same_label_ids
        episode["controls"]["controlled_recovery_task_outcomes"] = {"goal_2": expected}
        episodes.append(episode)

    return episodes


def _package(scene_id: str, scene_index: int, local_index: int, category: str,
             graph: dict[str, Any], anchors: list[dict[str, Any]],
             expected_branch: str | None = None,
             expected_recovery: str | None = None) -> dict[str, Any]:
    episode_id = f"{scene_id}_{local_index + 1:02d}"
    selected = {
        f"goal_{index + 1}": str(item["object"]["id"])
        for index, item in enumerate(anchors[:4])
    }
    if category == "ordered_conditional":
        selected["fallback_1"] = str(anchors[4]["object"]["id"])
    return {
        "format": "pre_map_vln.benchmark_episode.v1",
        "benchmark": "PRE-MAP-VLN-Bench-v1",
        "episode_id": episode_id,
        "scene_id": scene_id,
        "scene_index": scene_index,
        "episode_index": local_index,
        "category": category,
        "tags": ["multi_target", "long_instruction", "ordered", category],
        "task_graph": graph,
        "selected_objects": selected,
        "controls": {
            "stale_object_ids": [], "found_object_ids": [],
            "expected_branch": expected_branch,
            "expected_recovery": expected_recovery,
            "controlled_recovery_task_outcomes": {},
        },
    }


def validate_distribution(episodes: list[dict[str, Any]], scene_count: int) -> dict[str, Any]:
    categories = Counter(item["category"] for item in episodes)
    branches = Counter(
        item["controls"]["expected_branch"] for item in episodes
        if item["controls"]["expected_branch"] is not None
    )
    recoveries = Counter(
        item["controls"]["expected_recovery"] for item in episodes
        if item["controls"]["expected_recovery"] is not None
    )
    expected = {
        "ordered_spatial": 4 * scene_count,
        "ordered_conditional": 3 * scene_count,
        "ordered_recovery": 3 * scene_count,
    }
    if dict(categories) != expected:
        raise ValueError(f"invalid category distribution: {dict(categories)} != {expected}")
    if branches != Counter({"taken": 15, "skipped": 15}) and scene_count == 10:
        raise ValueError(f"invalid branch distribution: {dict(branches)}")
    if recoveries != Counter({"rediscovered": 2 * scene_count, "exhausted": scene_count}):
        raise ValueError(f"invalid recovery distribution: {dict(recoveries)}")
    return {
        "episode_count": len(episodes), "scene_count": scene_count,
        "categories": dict(categories), "branches": dict(branches),
        "recoveries": dict(recoveries),
    }
