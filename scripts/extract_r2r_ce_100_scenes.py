#!/usr/bin/env python3
"""Extract only the MP3D scenes referenced by OpenNav_R2R-CE_100."""

from __future__ import annotations

import argparse
import gzip
import json
import os
import shutil
import zipfile
from pathlib import Path, PurePosixPath


ROOT = Path(__file__).resolve().parents[1]
EPISODES = (
    ROOT
    / "third_party/Open-Nav/data/datasets/R2R_VLNCE_v1-2_preprocessed/"
    "val_unseen/OpenNav_R2R-CE_100_bertidx.json.gz"
)


def required_scenes() -> list[str]:
    with gzip.open(EPISODES, "rt", encoding="utf-8") as stream:
        episodes = json.load(stream)["episodes"]
    return sorted({Path(item["scene_id"]).stem for item in episodes})


def selected_members(archive: zipfile.ZipFile, scenes: set[str]):
    selected = []
    for info in archive.infolist():
        parts = PurePosixPath(info.filename).parts
        matches = [index for index, part in enumerate(parts) if part in scenes]
        if not matches:
            continue
        relative = Path(*parts[matches[0]:])
        if relative.is_absolute() or ".." in relative.parts:
            raise RuntimeError(f"unsafe archive member: {info.filename}")
        selected.append((info, relative))
    return selected


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, default=ROOT / "cache/mp3d_habitat.zip")
    parser.add_argument("--target", type=Path, default=ROOT / "data/scene_datasets/mp3d")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    scenes = required_scenes()
    with zipfile.ZipFile(args.archive) as archive:
        members = selected_members(archive, set(scenes))
        found = sorted({relative.parts[0] for _, relative in members})
        missing = sorted(set(scenes) - set(found))
        if missing:
            raise RuntimeError(f"archive lacks required scenes: {missing}")
        total_bytes = sum(info.file_size for info, _ in members if not info.is_dir())
        print(
            json.dumps(
                {
                    "required_scenes": scenes,
                    "members": len(members),
                    "uncompressed_bytes": total_bytes,
                    "target": str(args.target),
                    "dry_run": args.dry_run,
                },
                ensure_ascii=False,
            )
        )
        if args.dry_run:
            return
        args.target.mkdir(parents=True, exist_ok=True)
        records = []
        for info, relative in members:
            destination = args.target / relative
            if info.is_dir():
                destination.mkdir(parents=True, exist_ok=True)
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            if destination.is_file() and destination.stat().st_size == info.file_size:
                records.append({"path": str(relative), "size_bytes": info.file_size, "crc32": info.CRC})
                continue
            partial = destination.with_name(destination.name + ".part")
            with archive.open(info) as source, partial.open("wb") as target:
                shutil.copyfileobj(source, target, length=8 * 1024 * 1024)
            if partial.stat().st_size != info.file_size:
                raise RuntimeError(f"size mismatch after extracting {relative}")
            os.replace(partial, destination)
            records.append({"path": str(relative), "size_bytes": info.file_size, "crc32": info.CRC})

    absent_assets = []
    for scene in scenes:
        scene_dir = args.target / scene
        if not (scene_dir / f"{scene}.glb").is_file():
            absent_assets.append(f"{scene}/{scene}.glb")
        if not (scene_dir / f"{scene}.navmesh").is_file():
            absent_assets.append(f"{scene}/{scene}.navmesh")
    if absent_assets:
        raise RuntimeError(f"extracted bundle lacks Habitat assets: {absent_assets}")
    manifest = {
        "format": "pre_map_vln.mp3d_r2r_ce_100.v1",
        "source_archive": str(args.archive),
        "scenes": scenes,
        "file_count": len(records),
        "uncompressed_bytes": sum(item["size_bytes"] for item in records),
        "files": records,
    }
    manifest_path = ROOT / "data/baselines/mp3d_r2r_ce_100_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"extracted={len(scenes)} manifest={manifest_path}")


if __name__ == "__main__":
    main()
