#pragma once
// Surface-band Scan Context descriptor: for each (ring, sector) bin keep the
// max z among points whose range is within [r_min, r_min + band].
//
// Unlike the stock SCManager::makeScancontext (max z over ALL map points in
// a bin, including surfaces occluded from that viewpoint), this encodes only
// the first visible surface — the same occlusion structure a real single-view
// scan observes. DB cells and live scans then share one descriptor space,
// which is what makes full-map retrieval work with real sensor data.
//
// Used by both tools/build_map_scdb.cpp (DB side) and initial_align.cpp
// (query side); keep the parameters identical on both sides.
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <vector>

inline Eigen::MatrixXd make_surface_desc(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                         int nr = 40, int ns = 120,
                                         double rmax = 20.0, double band = 1.0)
{
    Eigen::MatrixXd desc = Eigen::MatrixXd::Constant(nr, ns, 0.0);
    std::vector<double> rmin(nr * ns, 1e18);
    for (const auto& p : cloud.points) {
        const double r = std::hypot(p.x, p.y);
        if (r <= 0.05 || r > rmax) continue;
        double ang = std::atan2(p.y, p.x);
        if (ang < 0) ang += 2 * M_PI;
        const int s = std::min(ns - 1, static_cast<int>(ang / (2 * M_PI) * ns));
        const int ring = std::min(nr - 1, static_cast<int>(r / rmax * nr));
        const int idx = ring * ns + s;
        if (r < rmin[idx]) rmin[idx] = r;
    }
    for (const auto& p : cloud.points) {
        const double r = std::hypot(p.x, p.y);
        if (r <= 0.05 || r > rmax) continue;
        double ang = std::atan2(p.y, p.x);
        if (ang < 0) ang += 2 * M_PI;
        const int s = std::min(ns - 1, static_cast<int>(ang / (2 * M_PI) * ns));
        const int ring = std::min(nr - 1, static_cast<int>(r / rmax * nr));
        const int idx = ring * ns + s;
        if (r <= rmin[idx] + band && p.z > desc(ring, s))
            desc(ring, s) = p.z;
    }
    return desc;
}
