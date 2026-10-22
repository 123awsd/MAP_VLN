#!/usr/bin/env python3
"""Maintain a small, evidence-linked experiment ledger. Never promotes a baseline."""
import argparse
import fcntl
import hashlib
import json
import re
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def describe(path):
    path = path.resolve()
    if not path.is_dir():
        raise FileNotFoundError('Trial removed or missing; preserve its archived record: ' + str(path))
    result = read_json(path / 'result.json')
    audit = read_json(path / 'rootfix_audit.json')
    execution = read_json(path / 'execution_contract_audit.json')
    volume = read_json(path / 'floor_volumes.json')
    manifest = read_json(path / 'trial_manifest.json')
    cfg_path = path / 'source_snapshot/config.yaml'
    cfg = yaml.safe_load(cfg_path.read_text()) if cfg_path.exists() else {}
    driver_path = path / 'run_simulator.sh'
    driver = driver_path.read_text() if driver_path.exists() else ''
    arguments = {}
    for key in ('duration', 'hz', 'time-scale', 'seed', 'collision-radius'):
        match = re.search(r'--' + key + r'\s+([\d.]+)', driver)
        if match:
            arguments[key] = float(match[1])
    arguments.setdefault('time-scale', 1.0)
    text = ANSI.sub('', (path / 'run.log').read_text(errors='replace')) if (path / 'run.log').exists() else ''
    lines = text.splitlines()
    episodes = []
    episode = None
    for number, line in enumerate(lines, 1):
        stamp = re.search(r'\[([0-9]{10}\.[0-9]+)\]', line)
        when = float(stamp[1]) if stamp else None
        selected = re.search(r'\[GoalLifecycle\] selected=\(([^)]+)\)', line)
        if selected:
            episode = {'target_xyz': [float(v) for v in selected[1].split(',')],
                       'start_ros_s': when, 'selected_log_line': number,
                       'evidence': [], 'status': 'no_explicit_completion_record'}
            episodes.append(episode)
        if episode is None:
            continue
        if any(key in line for key in ('[GoalLifecycle] release', '[ExecutionContract] request actual hold',
                                      'No path using', 'Blocking viewpoint', 'no usable conservative prefix',
                                      'No FREE straight prefix', 'Conditional A* prefix failed',
                                      'Collision detected on the trajectory before publishing')):
            episode['evidence'].append({'line': number, 'ros_s': when, 'message': line})
        if '[GoalLifecycle] release' in line:
            episode['status'] = 'completed' if 'complete=1' in line else 'released_without_arrival'
            episode['end_ros_s'] = when
    full_audit = bool(audit and (path / 'trial.bag').exists() and not (path / 'trial.bag.active').exists())
    interrupted = result.get('valid_full_trial') is False or 'interrupt' in str(result.get('termination', result.get('terminated', '')))
    state = 'interrupted' if interrupted else ('audited' if result and full_audit else 'running_or_incomplete')
    if not interrupted and not (path / 'run.log').exists() and not (path / 'planner_start.log').exists():
        state = 'prepared_not_started'
    duration = result.get('simulation_elapsed_seconds', audit.get('observed_sequence_duration_s'))
    short_test = bool(result.get('max_duration_seconds', 504) < 500)
    bathrooms = {key: audit.get(key) for key in ('bathroom_next_to_room2', 'bathroom_room9_upper_right')}
    entered_exited = {key: bool(value and any(run.get('exited') for run in value.get('entry_runs', [])))
                      for key, value in bathrooms.items()}
    # Entry alone and observed volume alone cannot establish non-regression.
    fixed = read_json(path / 'fixed_room_comparison.json')
    lower = [r for r in fixed.get('regions', [])
             if r.get('baseline_surface_reobserved_within_one_neighbor') is not None
             and r['baseline_surface_reobserved_within_one_neighbor'] < .95]
    failures = [e for e in episodes if e['status'] != 'completed' or
                any('hold' in v['message'] or 'prefix' in v['message'] for v in e['evidence'])]
    floors = volume.get('floors', [])
    return {'trial': path.name, 'storage': str(path), 'state': state,
            'parent_trial': manifest.get('parent_trial'),
            'simulator_arguments': arguments,
            'simulator_driver_sha256': hashlib.sha256(driver.encode()).hexdigest(),
            'source_snapshot_hashes_match_manifest': all(
                (path / 'source_snapshot' / name).exists() and
                hashlib.sha256((path / 'source_snapshot' / name).read_bytes()).hexdigest() == expected
                for name, expected in manifest.get('source_sha256', {}).items()) if manifest.get('source_sha256') else None,
            'duration_s': duration, 'evaluation_scope': 'short_test_only' if short_test else 'full_trial',
            'purpose': manifest.get('purpose'), 'termination': result,
            'config_sha256': hashlib.sha256(cfg_path.read_bytes()).hexdigest() if cfg_path.exists() else None,
            'source_sha256': manifest.get('source_sha256', {}),
            'configuration': cfg,
            'runtime_diagnostic_events': manifest.get('runtime_diagnostic_events', []),
            'path_m': audit.get('path_from_live_samples_m'), 'known_m3': volume.get('known_sum_m3'),
            'floor_known_m3': [v['known_m3'] for v in floors],
            'bathrooms': bathrooms, 'bathroom_entered_and_exited': entered_exited,
            'longest_position_pauses': audit.get('longest_position_pauses', []),
            'stationary_runs_at_least_5s': execution.get('stationary_runs'),
            'blocked_samples': audit.get('blocked_samples'), 'late_publication_count': execution.get('late_publication_count'),
            'observation_feedback_count': text.count('[ObservationOutcome]'),
            'goal_selection_count': len(episodes), 'goal_episodes_with_failures_or_missing_completion': failures,
            'fixed_regions_below_95_percent': lower if fixed else None,
            'verdict': 'short_test_only' if short_test and not interrupted else ('not_accepted' if interrupted or (full_audit and not all(entered_exited.values())) else 'requires_review'),
            'causal_limit': 'Log association is not causal proof. No explicit completion does not prove a specific rejection. Position pause may include yaw rotation. Surface support is not GT completeness.',
            'evidence_files': [str(f.relative_to(ROOT)) if f.is_relative_to(ROOT) else str(f)
                               for f in [path / n for n in ('trial_manifest.json', 'runtime_source_audit.json', 'rootfix_audit.json', 'execution_contract_audit.json', 'fixed_room_comparison.json', 'run.log', 'search_replay/snapshot.json')] if f.exists()]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trials', nargs='+', type=Path)
    args = parser.parse_args()
    updates = {p.resolve().name: describe(p) for p in args.trials}
    target = ROOT / 'docs/exploration_trial_index.json'
    with (ROOT / 'docs/.exploration_trial_index.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        records = read_json(target)
        records.update(updates)
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(records, indent=2, ensure_ascii=False))
        temp.replace(target)
        table = ['# 探索实验索引', '', '原始数据存data盘；本表由脚本生成。中止运行不作完整对比，进入房间不等于其他房间无退化。', '',
                 '| 实验 | 状态 | 时长 s | 体积 m³（1/2/3层） | 路程 m | 2号旁进出 | 9号右上进出 | 验收 |',
                 '|---|---|---|---|---|---|---|---|']
        for name, r in records.items():
            vols = '/'.join(f'{v:.1f}' for v in r['floor_known_m3']) or '待记录'
            dist = f"{r['path_m']:.1f}" if r['path_m'] is not None else '待记录'
            labels = [('是' if v else '否') if r['bathrooms'][k] is not None else '未验收'
                      for k, v in r['bathroom_entered_and_exited'].items()]
            duration = f"{r['duration_s']:.1f}" if r.get('duration_s') is not None else '未知'
            label = f"{name}（数据已清理）" if r.get('artifact_status') == 'deleted_after_summary_archival' else f"[{name}]({r['storage']})"
            table.append(f"| {label} | {r['state']} | {duration} | {vols} | {dist} | {labels[0]} | {labels[1]} | {r['verdict']} |")
        table.extend(['', '详细配置、源码哈希、失败目标及日志行号见 [JSON记录](exploration_trial_index.json)。',
                      '已完成试验的覆盖比较、停车审计等证据保存在各自目录；缺少证据的项不推断为通过。', ''])
        (ROOT / 'docs/EXPLORATION_TRIAL_INDEX.md').write_text('\n'.join(table))
    print(json.dumps({name: {'state': r['state'], 'verdict': r['verdict']} for name, r in updates.items()}, ensure_ascii=False))


if __name__ == '__main__':
    main()
