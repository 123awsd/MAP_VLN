"""Source-wiring regressions; runtime behavior is checked by offline previews."""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ScopeWiringTests(unittest.TestCase):
    def test_active_2d_gain_checks_polygon(self):
        source = (ROOT / 'runtime/falcon_gain_fix/frontier_finder.cpp').read_text()
        primary = source.split('void FrontierFinder::sampleViewpoints(')[1].split(
            'void FrontierFinder::sampleViewpoints3D(')[0]
        self.assertIn('!previewGeofenceAllowed(unknown_pos)', primary)
        self.assertIn('if (visib_nums[i] > 0)', primary)
        self.assertIn('if (lidar_gain) viewpoints[i].visib_num_ = visib_nums[i];', primary)

    def test_recovery_filters_before_nearest_limit(self):
        source = (ROOT / 'runtime/falcon_gain_fix/exploration_manager.cpp').read_text()
        recovery = source.split('auto addRecoveryCandidate =')[1].split(
            'const size_t max_recovery_candidates')[0]
        self.assertLess(recovery.index('!previewGeofenceAllowed(candidate)'),
                        recovery.index('recovery_candidates.push_back'))
        self.assertIn('map_server_->getESDF()->getVoxel(candidate).value < clearance', recovery)

    def test_global_grid_has_target_scope(self):
        source = (ROOT / 'runtime/falcon_gain_fix/hierarchical_grid.cpp').read_text()
        for name in ['isFree', 'isFreeInflated', 'isUnknown']:
            predicate = source.split('auto ' + name + ' =')[1].split('};')[0]
            self.assertIn('previewExplorationTargetAllowed', predicate)

    def test_unknown_start_exception_is_voxel_local(self):
        source = (ROOT / 'runtime/falcon_gain_fix/exploration_manager.cpp').read_text()
        spline_guard = source.split('if (preview_clearance > 0.0) {')[1].split('return SUCCEED;')[0]
        prefix_guard = source.split('bool ExplorationManager::planSafeAstarPrefix(')[1].split(
            'int ExplorationManager::updateFrontierStruct(')[0]
        for guard in [spline_guard, prefix_guard]:
            self.assertIn('voxel_mapping::OccupancyType::UNKNOWN', guard)
            self.assertIn('== start_cell.array()).all()', guard)
            self.assertIn('value < preview_clearance', guard)
            self.assertIn('previewGeofenceAllowed', guard)


if __name__ == '__main__':
    unittest.main()
