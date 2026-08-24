"""Read-only readiness inspection for the external VLN baselines."""

from __future__ import annotations

import gzip
import json
import os
import subprocess
from pathlib import Path
from typing import Any


OPENNAV_EPISODES = Path(
    "third_party/Open-Nav/data/datasets/R2R_VLNCE_v1-2_preprocessed/"
    "val_unseen/OpenNav_R2R-CE_100_bertidx.json.gz"
)
OPENNAV_WAYPOINT = Path("checkpoints/baselines/opennav/check_val_best_avg_wayscore")
OPENNAV_DDPPO = Path("checkpoints/baselines/opennav/gibson-2plus-resnet50.pth")


def _git_commit(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def _opennav_dataset(root: Path) -> tuple[int, list[str]]:
    path = root / OPENNAV_EPISODES
    if not path.is_file():
        return 0, []
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        payload = json.load(stream)
    episodes = payload.get("episodes", payload)
    scenes = sorted(
        {
            Path(str(episode.get("scene_id", ""))).stem
            for episode in episodes
            if episode.get("scene_id")
        }
    )
    return len(episodes), scenes


def _mp3d_scene_status(root: Path, scenes: list[str]) -> tuple[list[str], list[str]]:
    scene_root = root / "data/scene_datasets/mp3d"
    present, missing = [], []
    for scene in scenes:
        candidates = [scene_root / scene / f"{scene}.glb", scene_root / f"{scene}.glb"]
        (present if any(path.is_file() for path in candidates) else missing).append(scene)
    return present, missing


def _file(path: Path) -> dict[str, Any]:
    return {
        "path": str(path),
        "present": path.is_file(),
        "size_bytes": path.stat().st_size if path.is_file() else 0,
    }


def _patch_applied(repository: Path, patch: Path) -> bool:
    if not patch.is_file():
        return False
    result = subprocess.run(
        ["git", "-C", str(repository), "apply", "--reverse", "--check", str(patch)],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.returncode == 0


def _probe(python: Path, code: str, pythonpath: str, root: Path) -> dict[str, Any]:
    environment = os.environ.copy()
    environment.update(
        {
            "PYTHONPATH": pythonpath,
            "HF_HOME": str(root / "cache/huggingface"),
            "TORCH_HOME": str(root / "cache/torch"),
        }
    )
    result = subprocess.run(
        [str(python), "-c", code],
        cwd=root,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=60,
    )
    return {
        "passed": result.returncode == 0,
        "returncode": result.returncode,
        "stdout": result.stdout.strip()[-2000:],
        "stderr": result.stderr.strip()[-2000:],
    }


def inspect_external_baselines(root: Path, *, runtime: bool = False) -> dict[str, Any]:
    """Return auditable source/data/weight readiness without importing old stacks."""
    root = root.resolve()
    episode_count, scenes = _opennav_dataset(root)
    present_scenes, missing_scenes = _mp3d_scene_status(root, scenes)
    legacy_python = root / ".envs/vlnce_legacy/bin/python"
    sources = {
        name: {
            "path": str(root / "third_party" / directory),
            "commit": _git_commit(root / "third_party" / directory),
        }
        for name, directory in {
            "open_nav": "Open-Nav",
            "vln_zero": "VLN-Zero",
            "spatial_nav": "Spatial-X",
        }.items()
    }
    report = {
        "format": "pre_map_vln.external_baselines.v1",
        "sources": sources,
        "shared_legacy_environment": {
            "python": str(legacy_python),
            "present": legacy_python.is_file(),
            "reason": "Open-Nav and VLN-Zero require Habitat 0.1.7/Python 3.8",
        },
        "r2r_ce_100": {
            "episode_file": str(root / OPENNAV_EPISODES),
            "episode_count": episode_count,
            "scene_count": len(scenes),
            "scene_ids": scenes,
            "present_mp3d_scenes": present_scenes,
            "missing_mp3d_scenes": missing_scenes,
        },
        "weights": {
            "opennav_waypoint": _file(root / OPENNAV_WAYPOINT),
            "opennav_ddppo": _file(root / OPENNAV_DDPPO),
        },
        "controlled_ports": {
            "open_nav_qwen_owlv2": {
                "patch": str(root / "patches/Open-Nav-qwen-owlv2-port.patch"),
                "applied": _patch_applied(
                    root / "third_party/Open-Nav",
                    root / "patches/Open-Nav-qwen-owlv2-port.patch",
                ),
                "report_label": "Open-Nav (Qwen + shared OWLv2 port)",
            },
            "vln_zero_qwen": {
                "patch": str(root / "patches/VLN-Zero-qwen-port.patch"),
                "applied": _patch_applied(
                    root / "third_party/VLN-Zero",
                    root / "patches/VLN-Zero-qwen-port.patch",
                ),
                "report_label": "VLN-Zero (Qwen port)",
            },
        },
        "limitations": {
            "legacy_gpu": (
                "Upstream torch 2.4.1+cu121 lacks RTX 5060 Ti sm_120 kernels; "
                "use CPU for official smoke tests or bridge perception to the modern project process."
            ),
            "spatial_nav": (
                "Upstream has released raw agent code, but installation instructions, "
                "pre-exploration code, reconstruction code, and generated SSG data remain unavailable."
            ),
        },
    }
    report["ready"] = {
        "source_audit": all(item["commit"] for item in sources.values()),
        "opennav_without_scenes": (
            episode_count == 100
            and report["weights"]["opennav_waypoint"]["present"]
            and report["weights"]["opennav_ddppo"]["present"]
            and legacy_python.is_file()
        ),
        "controlled_ports": False,
        "r2r_ce_100_execution": episode_count == 100 and not missing_scenes,
        "spatialnav_official_end_to_end": False,
    }
    report["ready"]["controlled_ports"] = all(
        item["applied"] for item in report["controlled_ports"].values()
    )
    if runtime and legacy_python.is_file():
        legacy_habitat = root / "third_party/VLN-Zero/habitat-lab"
        report["runtime_probes"] = {
            "legacy_stack": _probe(
                legacy_python,
                "import habitat,habitat_sim,torch; "
                "print(habitat_sim.__version__, torch.__version__, torch.cuda.get_arch_list())",
                str(legacy_habitat),
                root,
            ),
            "open_nav_core": _probe(
                legacy_python,
                "import habitat_extensions,waypoint_prediction.TRM_net; print('ok')",
                f"{legacy_habitat}:{root / 'third_party/Open-Nav'}",
                root,
            ),
            "open_nav_full": _probe(
                legacy_python,
                "import vlnce_baselines; print('ok')",
                f"{legacy_habitat}:{root / 'third_party/Open-Nav'}",
                root,
            ),
            "vln_zero_core": _probe(
                legacy_python,
                "import VLN_CE.habitat_extensions,VLN_CE.vlnce_baselines; print('ok')",
                f"{legacy_habitat}:{root / 'third_party/VLN-Zero'}",
                root,
            ),
        }
    return report
