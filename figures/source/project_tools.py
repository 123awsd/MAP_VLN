"""Check manuscript inputs and create a portable LaTeX submission archive."""
from pathlib import Path
import argparse
import re
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def check():
    tex = (ROOT / 'manuscript/root.tex').read_text()
    names = re.findall(r'\\includegraphics(?:\[[^]]*\])?\{([^}]+)\}', tex)
    missing = [n for n in names if not (ROOT / 'figures/final' / n).is_file()]
    if missing:
        raise SystemExit('Missing figure files: ' + ', '.join(missing))
    actual = {p.name for p in (ROOT / 'figures/final').iterdir() if p.is_file()}
    if actual != set(names):
        raise SystemExit('final/ must contain exactly the installed manuscript figures.')
    for name in ('IEEEtran.cls', 'IEEEtran.bst', 'references.bib', '.latexmkrc', 'Makefile'):
        if not (ROOT / 'manuscript' / name).is_file():
            raise SystemExit('Missing manuscript dependency: ' + name)
    if r'\graphicspath{{../figures/final/}}' not in tex:
        raise SystemExit('Use the paper-local figures/final/ path.')
    if '/home/uav/' in tex:
        raise SystemExit('Absolute workspace path in manuscript source.')
    print(f'Checked {len(names)} figure references and portable manuscript inputs.')
    return names


def bundle():
    names = check()
    included = [Path('Makefile'), Path('README.md'),
                Path('figures/source/project_tools.py')]
    included += [Path('manuscript') / n for n in (
        'root.tex', 'references.bib', 'IEEEtran.cls', 'IEEEtran.bst', '.latexmkrc', 'Makefile')]
    included += [Path('figures/final') / n for n in names]
    dest = ROOT / 'manuscript/LogicFly_TASE_source.zip'
    tmp = dest.with_suffix('.tmp.zip')
    with zipfile.ZipFile(tmp, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name in included:
            archive.write(ROOT / name, name.as_posix())
    with zipfile.ZipFile(tmp) as archive:
        if archive.testzip() is not None:
            raise SystemExit('Archive verification failed.')
    tmp.replace(dest)
    print(dest)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['check', 'bundle'])
    args = parser.parse_args()
    if args.command == 'check':
        check()
    else:
        bundle()
