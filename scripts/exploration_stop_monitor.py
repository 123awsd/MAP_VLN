"""Experiment-only no-progress criterion; does not send planner commands."""
import math


def hold_evidence(frames):
    """Require measured position and consumed commands, not failure counts."""
    if not frames:
        return {'command_hold_fraction': 0.0, 'command_hold_without_progress': False}
    samples = []
    for frame in frames:
        command = frame.get('executed_command', {})
        velocity = command.get('velocity')
        samples.append(velocity is not None and len(velocity) == 3 and
                       all(math.isfinite(v) for v in velocity) and
                       math.sqrt(sum(v * v for v in velocity)) <= .01)
    fraction = sum(samples) / len(samples)
    return {'command_hold_fraction': fraction,
            'command_hold_without_progress': fraction >= .95}
