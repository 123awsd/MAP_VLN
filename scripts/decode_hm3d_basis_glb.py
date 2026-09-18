#!/usr/bin/env python3
"""Externalize an HM3D Basis GLB as glTF + decoded PNG textures for Blender."""
from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import magnum.trade as trade
import numpy as np
from PIL import Image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    raw = args.source.read_bytes()
    magic, version, total = struct.unpack_from("<4sII", raw, 0)
    if magic != b"glTF" or version != 2 or total != len(raw):
        raise ValueError("Expected a valid GLB 2.0 file")
    offset = 12
    chunks = []
    while offset < len(raw):
        size, kind = struct.unpack_from("<II", raw, offset)
        offset += 8
        chunks.append((kind, raw[offset:offset + size]))
        offset += size
    json_chunk = next(data for kind, data in chunks if kind == 0x4E4F534A)
    bin_chunk = next(data for kind, data in chunks if kind == 0x004E4942)
    document = json.loads(json_chunk.rstrip(b" \t\r\n\0"))
    binary_name = "scene.bin"
    (args.output_dir / binary_name).write_bytes(bin_chunk)
    document["buffers"][0]["uri"] = binary_name

    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    importer.open_file(str(args.source))
    if importer.image2d_count != len(document.get("images", [])):
        raise RuntimeError("Decoded image count does not match glTF image table")
    for index in range(importer.image2d_count):
        name = f"texture_{index:03d}.png"
        target = args.output_dir / name
        if target.exists():
            try:
                with Image.open(target) as existing:
                    existing.verify()
                print(f"reusing {index + 1}/{importer.image2d_count}", flush=True)
                document["images"][index] = {"uri": name}
                continue
            except Exception:
                target.unlink()
        image = importer.image2d(index)
        width, height = map(int, image.size)
        pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(height, width, 4)
        Image.fromarray(pixels, "RGBA").save(target, compress_level=1)
        document["images"][index] = {"uri": name}
        if index % 10 == 0 or index + 1 == importer.image2d_count:
            print(f"decoded {index + 1}/{importer.image2d_count}", flush=True)

    for texture in document.get("textures", []):
        extensions = texture.get("extensions", {})
        basis_key = next((key for key in ("KHR_texture_basisu", "GOOGLE_texture_basis")
                          if key in extensions), None)
        basis = extensions.get(basis_key) if basis_key else None
        if basis is not None:
            texture["source"] = basis["source"]
            del texture["extensions"][basis_key]
            if not texture["extensions"]:
                del texture["extensions"]
    for key in ("extensionsUsed", "extensionsRequired"):
        if key in document:
            document[key] = [item for item in document[key]
                             if item not in ("KHR_texture_basisu", "GOOGLE_texture_basis")]
            if not document[key]:
                del document[key]
    output = args.output_dir / "scene.gltf"
    output.write_text(json.dumps(document, separators=(",", ":")))
    print(output)


if __name__ == "__main__":
    main()
