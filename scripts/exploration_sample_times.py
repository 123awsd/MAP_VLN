"""Read actual sample times when available, otherwise label nominal-time estimates."""
import re
from pathlib import Path
import numpy as np


def sample_times(rows, trial):
    for key in ('capture_stamp', 'monitor_wall_time'):
        if rows and all(key in row for row in rows):
            values = np.asarray([row[key] for row in rows], dtype=float)
            return values - values[0], key
    script = Path(trial) / 'run_simulator.sh'
    match = re.search(r'--hz\s+([\d.]+)', script.read_text()) if script.exists() else None
    hz = float(match[1]) if match else 5.0
    values = np.asarray([row['sequence'] / hz for row in rows], dtype=float)
    return values - values[0], f'nominal_sequence_time_at_{hz:g}Hz_not_capture_time'
