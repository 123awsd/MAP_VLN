"""Move closed-node rejection ahead of redundant geometry reads in cost searches.

The two UNKNOWN-permitting search variants feed HGrid. Execution FREE searches,
map contents, budgets, neighbour order, heuristics and safety predicates stay intact.
"""
from pathlib import Path
import hashlib, json, shutil


def transform(text):
    for name, following in [('searchUnknown', 'searchUnknownBBox'),
                            ('searchUnknownBBox', 'searchUnknownOnlyBBox')]:
        begin = text.index(f'int Astar::{name}(')
        end = text.index(f'int Astar::{following}(', begin)
        part = text[begin:end]
        a = part.index('          // Check not in close set\n')
        b = part.index('          AstarNode::Ptr neighbor;', a)
        block = part[a:b]
        assert 'posToIndex(nbr_pos, nbr_idx);' in block
        part = part[:a] + part[b:]
        anchor = '          nbr_pos = cur_pos + step;\n'
        assert part.count(anchor) == 1
        part = part.replace(anchor, anchor + block)
        text = text[:begin] + part + text[end:]
    return text


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    base = root/'runtime/consistency_validation_20261015'
    src, dst = base/'A3_observation_lifecycle', base/'A3_astar_closed_first'
    if dst.exists():
        raise SystemExit('fresh candidate required')
    shutil.copytree(src, dst)
    (dst/'astar.cpp').write_text(transform((src/'astar.cpp').read_text()))
    # Freeze the dependency explicitly, including unchanged upstream HGrid.
    dep = base/'shared_dependencies/hierarchical_grid.cpp'
    assert hashlib.sha256(dep.read_bytes()).hexdigest() == '2fca25380ba13bd1551ca5b767b8f079ec5890a65cc82a29adcf524765b08896'
    shutil.copy2(dep, dst/'hierarchical_grid.cpp')
    manifest = {'base': 'A3_observation_lifecycle',
                'purpose': 'Reject already-closed nodes before geometry reads in searchUnknown and searchUnknownBBox only',
                'source_sha256': {str(p.relative_to(dst)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in dst.rglob('*') if p.is_file() and '__pycache__' not in p.parts}}
    (base/'A3_astar_closed_first_manifest.json').write_text(json.dumps(manifest, indent=2))
    print(dst)
