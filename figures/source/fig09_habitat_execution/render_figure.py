#!/usr/bin/python3
"""Compose recorded Habitat keyframes into a compact, traceable paper figure.

Uses the system Python's cairo + Rsvg. Composition runs no inference.
Scene pixels come from native high-resolution replay at exact recorded poses.
"""
from __future__ import annotations

import base64
import hashlib
import html
import json
from pathlib import Path

import cairo
import gi

gi.require_version("Rsvg", "2.0")
from gi.repository import Gio, GLib, Rsvg

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1] / "PRE_MAP_VLN"
RUNS = REPO / "outputs/stage2_3d/tasks"
W, H = 2400, 1124
M, GAP, IW, IH = 30, 12, 380, 285
C = {
    "ink": "#000000", "muted": "#626D76", "line": "#DDE2E5",
    "microwave": "#287A91", "paper": "#32876F",
    "table": "#8062AA", "tv": "#BD713D",
    "miss": "#BE6A35", "found": "#267C65", "motion": "#73818B",
}
PARTS: list[str] = []
EVIDENCE: list[dict] = []
TEXT_BOUNDS: list[dict] = []
_MEASURE = cairo.Context(cairo.ImageSurface(cairo.FORMAT_ARGB32, 1, 1))
REPLAY_PATH = ROOT / "keyframes/render_manifest.json"
REPLAY = json.loads(REPLAY_PATH.read_text())


def load(path: Path) -> dict:
    return json.loads(path.read_text())


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def text(x, y, value, size=32, color=None, bold=False, serif=False, anchor="start"):
    family = "TeX Gyre Termes" if serif else "TeX Gyre Heros"
    _MEASURE.select_font_face(family, cairo.FONT_SLANT_NORMAL,
                              cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
    _MEASURE.set_font_size(size)
    ext = _MEASURE.text_extents(value)
    width = ext.x_advance
    left = x if anchor == "start" else x - width / 2 if anchor == "middle" else x - width
    assert left >= 0 and left + width <= W + 1, (value, left, width)
    TEXT_BOUNDS.append({"text": value, "x": left, "y": y, "width": width})
    PARTS.append(
        f'<text xml:space="preserve" x="{x}" y="{y}" font-family="{family}" font-size="{size}" '
        f'font-weight="{700 if bold else 400}" fill="{color or C["ink"]}" '
        f'text-anchor="{anchor}">{html.escape(value)}</text>'
    )
    return width


def sentence(y, spans):
    full = "".join(value for value, _ in spans)
    width = 0
    for value, bold in spans:
        _MEASURE.select_font_face("TeX Gyre Termes", cairo.FONT_SLANT_NORMAL,
                                 cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
        _MEASURE.set_font_size(36)
        width += _MEASURE.text_extents(value).x_advance
    assert width <= W - 2*M + 1, (y, width)
    TEXT_BOUNDS.append({"text": full, "x": (W-width)/2, "y": y, "width": width})
    # Black throughout; only search targets are bold. Keep continuous shaping.
    styled = "".join(f'<tspan font-weight="{700 if bold else 400}">{html.escape(value)}</tspan>'
                     for value, bold in spans)
    PARTS.append(f'<text xml:space="preserve" x="{W/2}" y="{y}" '
                 f'font-family="TeX Gyre Termes" font-size="36" fill="#000000" '
                 f'text-anchor="middle">{styled}</text>')


def line(x1, y1, x2, y2, color, width=1):
    PARTS.append(f'<path d="M{x1} {y1}L{x2} {y2}" stroke="{color}" stroke-width="{width}" fill="none"/>')


def episode(name: str):
    directory = RUNS / name
    graph = load(directory / "task_graph.json")
    trace = load(directory / "execution/habitat_execution.json")
    assert trace["verification_mode"] == "owlv2"
    assert trace["status"] == "complete"
    recovery = trace.get("semantic_recovery", {})
    assert not recovery.get("controlled_stale_object_ids")
    assert not recovery.get("controlled_found_object_ids")
    for obs in trace["observations"]:
        assert not obs["verification"].get("controlled_stale_map")
        assert not obs["verification"].get("controlled_found")
    record = {
        "run": str(directory.relative_to(REPO)),
        "scene": trace["scene"],
        "task_graph_sha256": sha(directory / "task_graph.json"),
        "execution_log_sha256": sha(directory / "execution/habitat_execution.json"),
        "instruction_original": graph["instruction"],
        "verification_mode": trace["verification_mode"],
        "status": trace["status"],
        "controlled_outcomes": False,
        "recovery_planner": recovery.get("planner"),
        "selected_frames": [],
    }
    EVIDENCE.append(record)
    return directory / "execution", trace, record


def panel(execution, trace, record, col, y, label, color, *, obs=None, frame=None, detection_index=None):
    assert (obs is None) != (frame is None)
    x = M + col * (IW + GAP)
    observation = trace["observations"][obs] if obs is not None else None
    if observation:
        path = execution / observation["rgb"]
        index = observation["frame_index"]
    else:
        path = execution / "frames" / f"frame_{frame:06d}.jpg"
        index = frame
    assert path.is_file()
    pose = observation["pose"] if observation else trace["trajectory_xyz_yaw"][index + 1]
    replay_episode = next(item for item in REPLAY["episodes"] if item["run"] == execution.parent.name)
    assert replay_episode["trace_sha256"] == record["execution_log_sha256"]
    replay_key = f"obs:{obs}" if observation else f"frame:{frame}"
    replay = next(item for item in replay_episode["frames"] if item["key"] == replay_key)
    assert replay["pose"] == pose
    assert replay["source_sha256"] == sha(path)
    hd_path = ROOT / replay["image"]
    assert replay["image_sha256"] == sha(hd_path)
    data = base64.b64encode(hd_path.read_bytes()).decode("ascii")
    PARTS.append(f'<image x="{x}" y="{y}" width="{IW}" height="{IH}" '
                 f'preserveAspectRatio="xMidYMid meet" href="data:image/png;base64,{data}"/>')
    PARTS.append(f'<rect x="{x}" y="{y}" width="{IW}" height="{IH}" '
                 f'fill="none" stroke="{C["line"]}" stroke-width="1"/>')

    boxes = []
    if observation and observation["verification_outcome"] == "found":
        detections = observation["verification"].get("owlv2", {}).get("detections", [])
        # Keep the exact recorded coordinates. A reviewed detection can be
        # selected to show a loose cushion instead of an upholstered seat.
        if detections:
            detection = (max(detections, key=lambda d: d["score"]) if detection_index is None
                         else detections[detection_index])
            bx1, by1, bx2, by2 = detection["bbox_xyxy"]
            sx, sy = IW / 640, IH / 480
            bx1, bx2 = x + bx1 * sx, x + bx2 * sx
            by1, by2 = y + by1 * sy, y + by2 * sy
            length = min(12, (bx2-bx1)/3, (by2-by1)/3)
            for ax, ay, dx, dy in ((bx1,by1,1,1),(bx2,by1,-1,1),(bx1,by2,1,-1),(bx2,by2,-1,-1)):
                PARTS.append(f'<path d="M{ax+dx*length} {ay}H{ax}V{ay+dy*length}" '
                             f'fill="none" stroke="{color}" stroke-width="2.5"/>')
            boxes.append(detection)
    frame_record = {
        "image": str(path.relative_to(REPO)), "image_sha256": sha(path),
        "display_image": replay["image"], "display_image_sha256": replay["image_sha256"],
        "display_image_resolution_wh": REPLAY["resolution_wh"],
        "display_image_source": "Native Habitat replay at the exact logged camera pose and FoV",
        "frame_index": index, "observation_sequence": obs,
        "selection_description": label, "frame_caption_displayed": False,
        "detector_boxes": boxes,
        "detector_box_selection": ("highest recorded score" if detection_index is None
                                   else f"visually reviewed recorded detection {detection_index}; coordinates unchanged"),
    }
    if observation:
        frame_record.update({key: observation.get(key) for key in
                             ("task_id", "outcome", "verification_outcome", "pose")})
        frame_record["recovery_reason"] = observation.get("recovery", {}).get("reason")
        frame_record["location_exhausted"] = observation.get("recovery", {}).get("location_exhausted")
    else:
        frame_record["pose"] = pose
    record["selected_frames"].append(frame_record)


def build():
    PARTS.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">')
    PARTS.append(f'<rect width="{W}" height="{H}" fill="white"/>')
    PARTS.append('<g transform="translate(0 12)">')
    execution, trace, record = episode("00337_all_found_showcase_v1_20260830")
    sentence(41, [
        ("“On Floor 2, check the ", False), ("microwave", True),
        (" in the kitchen, the ", False), ("coffee table", True),
        (" in the living room,", False),
    ])
    sentence(82, [
        ("and ", False), ("toilet paper", True),
        (" in the bathroom. Also check the ", False), ("TV", True),
        (" in the Floor 3 living room. Choose any order.”", False),
    ])
    record["instruction_display_note"] = "Short English summary for display. All target room and floor scopes retained; fine anchor qualifiers and social preamble omitted. The original complete request is retained in this manifest."
    panel(execution,trace,record,0,104,"Floor 2",C["motion"],frame=150)
    panel(execution,trace,record,1,104,"Microwave",C["microwave"],obs=0)
    panel(execution,trace,record,2,104,"Toilet paper",C["paper"],obs=1)
    panel(execution,trace,record,3,104,"Coffee table",C["table"],obs=2)
    panel(execution,trace,record,4,104,"Floor 3",C["motion"],frame=900)
    panel(execution,trace,record,5,104,"TV",C["tv"],obs=3)

    execution, trace, record = episode("00166_cushion_recovery_20261024")
    sentence(438, [
        ("“Find a ", False), ("cushion", True),
        (" on the bedroom armchair; if it is missing, search other rooms until found.”", False),
    ])
    record["instruction_display_note"] = "The actual English input, enclosed in quotation marks. Recovery stays within one task, rather than an explicit conditional-task branch."
    record["task_and_recovery_input"] = "Assistant-authored offline task graph and recovery hypotheses, authorized by the user for visualization; no Qwen call. Candidate generation, execution and OWLv2 verification ran locally."
    record["run_manifest"] = str((execution.parent / "run_manifest.json").relative_to(REPO))
    assert len(trace["semantic_recovery"]["events"]) == 1
    assert len({obs["task_id"] for obs in trace["observations"]}) == 1
    panel(execution,trace,record,0,460,"Enter the bedroom",C["motion"],frame=205)
    panel(execution,trace,record,1,460,"Armchair: first negative observation",C["miss"],obs=0)
    panel(execution,trace,record,2,460,"Armchair: local viewpoint change",C["motion"],frame=245)
    panel(execution,trace,record,3,460,"Search another room after local exhaustion",C["motion"],frame=380)
    panel(execution,trace,record,4,460,"Approach the living-room sofa",C["motion"],frame=420)
    panel(execution,trace,record,5,460,"Cushion found",C["found"],obs=3,detection_index=2)
    record["omitted_terminal_observation_sequences"] = [1,2]
    record["recovery_trigger"] = trace["semantic_recovery"]["events"][0]

    execution, trace, record = episode("00070_pillow_conditional_20261024")
    sentence(794, [
        ("“Check the study chair for a ", False), ("pillow", True),
        (". If one is found, check the bedroom bed for another ", False), ("pillow", True),
        (".”", False),
    ])
    record["instruction_display_note"] = "The actual English input, enclosed in quotation marks. The second task is initially inactive and is activated only by a positive first-task result."
    record["task_input"] = "Assistant-authored offline task graph, authorized by the user; no Qwen call. Conditional activation, candidate generation, execution and OWLv2 verification ran locally."
    record["run_manifest"] = str((execution.parent / "run_manifest.json").relative_to(REPO))
    finished = [event for event in trace["events"] if event["event"] == "task_finished"]
    assert finished[0]["outcome"] == "found"
    assert {"task_id": "check_bedroom_pillow", "status": "pending", "reason": "condition_matched"} in finished[0]["state_changes"]
    assert not trace["semantic_recovery"]["enabled"]
    record["conditional_trigger"] = finished[0]
    panel(execution,trace,record,0,816,"Enter the study",C["motion"],frame=160)
    panel(execution,trace,record,1,816,"Approach the study chair",C["motion"],frame=210)
    panel(execution,trace,record,2,816,"Pillow found; activate the bedroom task",C["found"],obs=0)
    panel(execution,trace,record,3,816,"Enter the bedroom after condition matched",C["motion"],frame=310)
    panel(execution,trace,record,4,816,"Approach the bedroom bed",C["motion"],frame=324)
    panel(execution,trace,record,5,816,"Another pillow found",C["found"],obs=1)
    for record in EVIDENCE:
        indexes = [f["frame_index"] for f in record["selected_frames"]]
        assert indexes == sorted(indexes)
    PARTS.append("</g></svg>")
    return "\n".join(PARTS).encode("utf-8")


def main():
    svg = build()
    stem = ROOT / "habitat_long_instruction_en"
    stem.with_suffix(".svg").write_bytes(svg)
    # The self-contained SVG embeds eighteen lossless HD images.
    stream = Gio.MemoryInputStream.new_from_bytes(GLib.Bytes.new(svg))
    handle = Rsvg.Handle.new_from_stream_sync(stream, None, Rsvg.HandleFlags.FLAG_UNLIMITED, None)
    view = Rsvg.Rectangle()
    view.x, view.y, view.width, view.height = 0,0,W,H
    pdf = cairo.PDFSurface(str(stem.with_suffix(".pdf")),516,H*516/W)
    pdf.set_metadata(cairo.PDF_METADATA_TITLE,"Long-instruction execution in Habitat")
    ctx = cairo.Context(pdf)
    ctx.scale(516/W,516/W)
    handle.render_document(ctx,view)
    pdf.finish()
    image = cairo.ImageSurface(cairo.FORMAT_ARGB32,W,H)
    handle.render_document(cairo.Context(image),view)
    image.write_to_png(str(stem.with_suffix(".png")))
    image.finish()
    (ROOT/"provenance.json").write_text(json.dumps({
        "kind":"Three-scene qualitative preview: archived multi-task episode; newly executed offline-reasoned recovery and explicit conditional episodes",
        "scenes":["00337-CFVBbU9Rsyb", "00166-RaYrxWt5pR1", "00070-w7QyjJ3H9Bp"], "paper_updated":False,
        "typography":"Centered quoted instructions in black; only target nouns are bold. No a/b/c labels or frame captions.",
        "presentation":"Complete 4:3 camera fields of view, natively replayed at 1600x1200 from recorded poses; no cropping, retouching, upscaling, artificial assets or model-generated pixels. Colored corner marks use the original recorded 640x480 OWLv2 detections, mapped to the same FoV.",
        "ordering":"Left to right in actual execution order. Selected frames, not equally spaced timestamps.",
        "replay_manifest_sha256":sha(REPLAY_PATH),
        "episodes":EVIDENCE,
        "generator_sha256":sha(Path(__file__)),
        "outputs_sha256":{s:sha(stem.with_suffix(s)) for s in (".svg",".pdf",".png")},
    },ensure_ascii=False,indent=2)+"\n")
    for suffix in (".pdf",".png",".svg"):
        print(stem.with_suffix(suffix))


if __name__ == "__main__":
    main()
