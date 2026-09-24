import csv
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'real_fly/stage2_runtime/scripts/timestamped_rgb_encoder.py'
spec = importlib.util.spec_from_file_location('encoder', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def boxes(data, start=0, end=None):
    end = len(data) if end is None else end
    while start + 8 <= end:
        size, kind = struct.unpack_from('>I4s', data, start)
        if size < 8:
            raise ValueError('Invalid test MP4 box')
        body = start + 8
        yield kind, data[body:start + size]
        if kind in (b'moov', b'trak', b'mdia', b'minf', b'stbl'):
            yield from boxes(data, body, start + size)
        start += size


class TimingTest(unittest.TestCase):
    def test_invalid_clock_fails(self):
        for stamps in ((0,), (100, 100), (100, 99)):
            clock = module.SourceClock()
            with self.assertRaises(ValueError):
                for stamp in stamps:
                    clock.relative_ns(stamp)

    def test_dropped_frames_keep_real_mp4_timeline(self):
        # Only four frames arrive across one second, despite a nominal 30 Hz.
        stamps = [1_000_000_000, 1_033_333_333, 1_500_000_000, 2_000_000_000]
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / 'test.mp4')
            encoder = module.TimestampedEncoder(output, 64, 48, 30, 500)
            for stamp in stamps:
                encoder.push(bytes(64 * 48 * 3), stamp)
            encoder.close()
            contents = dict(boxes(Path(output).read_bytes()))
            timescale = struct.unpack_from('>I', contents[b'mdhd'], 12)[0]
            stts = contents[b'stts']
            count = struct.unpack_from('>I', stts, 4)[0]
            deltas = []
            for i in range(count):
                repeat, delta = struct.unpack_from('>II', stts, 8 + i * 8)
                deltas.extend([delta / timescale] * repeat)
            self.assertEqual(len(deltas), 4)
            for actual, expected in zip(deltas, (1/30, 14/30, 0.5, 1/30)):
                self.assertAlmostEqual(actual, expected, delta=0.001)
            self.assertAlmostEqual(sum(deltas), 1 + 1/30, delta=0.001)
            with open(output + '.timestamps.csv') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([int(row['source_stamp_ns']) for row in rows], stamps)
            with self.assertRaises(FileExistsError):
                module.TimestampedEncoder(output, 64, 48, 30, 500)


if __name__ == '__main__':
    unittest.main()
