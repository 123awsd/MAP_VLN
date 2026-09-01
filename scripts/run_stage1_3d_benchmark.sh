#!/usr/bin/env bash
set -u -o pipefail

# Run a scene list round-by-round with the standard 3-D Stage1 entry point,
# then export each closed compact Bag and render truth comparison figures.
# Existing complete artifacts are reused, making an interrupted batch resumable.

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
batch_name="${1:?usage: $0 BATCH_NAME [DURATION] [HZ] [RECORD_EVERY] SCENE_ID...}"
duration="${2:-600}"
hz="${3:-5}"
record_every="${4:-5}"
shift $(( $# >= 4 ? 4 : $# ))
scene_ids=("$@")
seeds=(7 17 27 37)

if [[ ! "$batch_name" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "batch name contains unsupported characters: $batch_name" >&2
  exit 2
fi
if ! [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ \
  && "$hz" =~ ^[0-9]+([.][0-9]+)?$ \
  && "$record_every" =~ ^[1-9][0-9]*$ ]]; then
  echo "duration and hz must be positive numbers; record_every must be a positive integer" >&2
  exit 2
fi
if [[ "${#scene_ids[@]}" -eq 0 ]]; then
  echo "at least one scene ID is required" >&2
  exit 2
fi
for scene_id in "${scene_ids[@]}"; do
  if [[ ! "$scene_id" =~ ^[0-9]{5}-[A-Za-z0-9]+$ ]]; then
    echo "scene id must look like 00166-RaYrxWt5pR1: $scene_id" >&2
    exit 2
  fi
done

cd "$root_dir"
overall=0
for round_offset in "${!seeds[@]}"; do
  round_number=$((round_offset + 1))
  round_label="$(printf '%02d' "$round_number")"
  seed="${seeds[$round_offset]}"
  echo "========== ROUND $round_label/04 seed=$seed =========="

  for scene_id in "${scene_ids[@]}"; do
    scene_number="${scene_id%%-*}"
    scene_token="${scene_id#*-}"
    run_name="${batch_name}_r${round_label}_seed${seed}"
    run_dir="$root_dir/outputs/stage1_3d/$scene_id/$run_name"
    bag_path="$root_dir/outputs/bags/hm3d_stage1_3d_${scene_number}_${run_name}_compact.bag"
    scene_path="${PRE_MAP_VLN_HM3D_TRAIN_ROOT:-/shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train}/$scene_id/$scene_token.basis.glb"
    comparison_dir="$root_dir/outputs/scenes/$scene_id/stage1/comparisons/$run_name"

    echo "----- $scene_id | $run_name -----"
    if [[ ! -d "$run_dir" && ! -f "$bag_path" ]]; then
      if ! "$root_dir/scripts/run_hm3d_stage1_3d.sh" \
        "$scene_id" "$run_name" "$duration" "$hz" "$record_every" \
        false compact "$seed"; then
        echo "run did not enter legal FINISH; retaining closed Bag for evaluation"
      fi
    elif [[ -d "$run_dir" && -f "$bag_path" ]]; then
      echo "run and Bag already exist; resuming from postprocess"
    else
      echo "inconsistent existing output; need both run directory and Bag: $run_dir / $bag_path" >&2
      overall=1
      continue
    fi

    if [[ ! -f "$bag_path" ]]; then
      echo "closed Bag missing; cannot evaluate this run" >&2
      overall=1
      continue
    fi
    if [[ ! -d "$run_dir/bag_export" ]]; then
      if ! "$root_dir/scripts/export_stage1_run.sh" "$run_dir" "$bag_path"; then
        echo "Bag export failed: $scene_id $run_name" >&2
        overall=1
        continue
      fi
    else
      echo "bag_export already exists; skipping export"
    fi

    mkdir -p "$comparison_dir"
    if ! "$root_dir/.envs/habitat/bin/python" \
      "$root_dir/scripts/plot_stage1_3d_truth_comparison.py" \
      --run-dir "$run_dir" \
      --scene "$scene_path" \
      --output "$comparison_dir/truth_vs_falcon_3d.png" \
      --floor-output "$comparison_dir/floor_comparison.png"; then
      echo "comparison rendering failed: $scene_id $run_name" >&2
      overall=1
      continue
    fi

    "$root_dir/.envs/habitat/bin/python" \
      "$root_dir/scripts/refresh_scene_output_index.py" \
      --scene-id "$scene_id" || overall=1
    echo "comparison: $comparison_dir"
  done
done

echo "========== BATCH FINISHED: $batch_name (exit=$overall) =========="
exit "$overall"
