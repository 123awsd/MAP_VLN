#!/usr/bin/env python3
"""Load one official Open-Nav R2R-CE episode and render its sensors.

Run this through ``scripts/run_vlnce_legacy.sh open-nav``.  It intentionally
does not call a language model or execute a navigation policy.
"""

from __future__ import annotations

import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OPEN_NAV = ROOT / "third_party/Open-Nav"


def main() -> None:
    os.chdir(OPEN_NAV)

    import habitat
    import habitat_extensions  # noqa: F401 - registers VLN-CE dataset/task
    from vlnce_baselines.config.default import get_config

    config = get_config("run_OpenNav.yaml")
    config.defrost()
    config.TASK_CONFIG.ENVIRONMENT.MAX_EPISODE_STEPS = 2
    config.TASK_CONFIG.DATASET.EPISODES_TO_LOAD = 1
    config.freeze()

    dataset = habitat.make_dataset(
        config.TASK_CONFIG.DATASET.TYPE,
        config=config.TASK_CONFIG.DATASET,
    )
    episode = dataset.episodes[0]
    with habitat.Env(config=config.TASK_CONFIG, dataset=dataset) as environment:
        observations = environment.reset()
        state = environment.sim.get_agent_state()
        report = {
            "format": "pre_map_vln.r2r_ce_scene_smoke.v1",
            "episode_id": str(episode.episode_id),
            "scene_id": episode.scene_id,
            "instruction": episode.instruction.instruction_text,
            "sensor_shapes": {
                key: list(value.shape) if hasattr(value, "shape") else None
                for key, value in observations.items()
            },
            "agent_position": [float(value) for value in state.position],
            "passed": "rgb" in observations and "depth" in observations,
        }

    output = ROOT / "outputs/baselines/r2r_ce_scene_smoke.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit("RGB/depth observations were not produced")


if __name__ == "__main__":
    main()
