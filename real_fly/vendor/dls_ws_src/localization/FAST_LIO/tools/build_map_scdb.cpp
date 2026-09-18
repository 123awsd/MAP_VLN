// build_map_scdb.cpp — offline Scan Context descriptor DB builder for a venue
// PCD. The upgraded initial_align node loads the .scdb to search the whole
// map for the drone's initial pose (no more origin-only zone).
//
// Built by the fast_lio catkin package (devel/lib/fast_lio/build_map_scdb).
// Entry point: ./build_map_scdb.sh MAP.pcd [grid_step_m=1.0] [lidar_height_m=0.5]
// Usage (direct):
//   rosrun fast_lio build_map_scdb <map.pcd> <out.scdb> [grid_step_m=1.0] [lidar_height_m=0.5]
//
// File format (binary, little-endian host):
//   char   magic[5]     = "SCDB1"
//   uint32 version      = 1
//   float  grid_step
//   uint32 count
//   per entry:
//     float x, y
//     float desc[RING*SECTOR]   row-major (ring, sector), max-z encoding
//     float ringkey[RING]       rowwise mean of desc
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "scancontext/Scancontext.h"
#include "scancontext/surface_desc.h"

namespace {

constexpr int    PC_NUM_RING   = 40;
constexpr int    PC_NUM_SECTOR = 120;
constexpr double PC_MAX_RADIUS = 20.0;

pcl::PointCloud<pcl::PointXYZ>::Ptr load_xyz(const std::string& path) {
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *out) == -1) {
    std::cerr << "[scdb] cannot load " << path << std::endl;
    std::exit(1);
  }
  return out;
}

// Crop to a horizontal disk of radius r around (cx, cy). Points are stored in
// a frame whose origin is (cx, cy, 0) and z is relative to the LIDAR height
// (matching what the live scan sees in the lidar frame).
pcl::PointCloud<pcl::PointXYZ>::Ptr crop_lidar_frame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double cx, double cy, double r, double lidar_height) {
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  out->reserve(cloud->size());
  // NOTE: no vertical FOV band here. The drone may start elevated (table,
  // box, ...) so its real scan band is shifted in z; a fixed band on the DB
  // side would slice a different part of the scene and destroy descriptor
  // matching (NX field finding, 2026-08). The surface-band encoding
  // (surface_desc.h) already limits bins to the nearest surface.
  for (const auto& p : cloud->points) {
    const double dx = p.x - cx, dy = p.y - cy;
    const double dxy = std::hypot(dx, dy);
    if (dxy > r) continue;
    out->points.emplace_back(dx, dy, p.z - lidar_height);
  }
  out->width = out->points.size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double leaf) {
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(cloud);
  vg.filter(*out);
  return out;
}

} // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0]
              << " <map.pcd> <out.scdb> [grid_step_m=1.0] [lidar_height_m=0.5]" << std::endl;
    return 2;
  }
  const std::string map_path = argv[1];
  const std::string out_path = argv[2];
  const double grid_step = argc > 3 ? std::atof(argv[3]) : 1.0;
  const double lidar_height = argc > 4 ? std::atof(argv[4]) : 0.5;

  const auto map = load_xyz(map_path);
  std::cout << "[scdb] map=" << map_path << " points=" << map->size() << std::endl;

  double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
  for (const auto& p : map->points) {
    min_x = std::min(min_x, static_cast<double>(p.x));
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }
  std::cout << "[scdb] extent x[" << min_x << ", " << max_x
            << "] y[" << min_y << ", " << max_y << "]" << std::endl;

  SCManager sc_manager;
  sc_manager.PC_NUM_RING = PC_NUM_RING;
  sc_manager.PC_NUM_SECTOR = PC_NUM_SECTOR;
  sc_manager.PC_MAX_RADIUS = PC_MAX_RADIUS;
  sc_manager.PC_UNIT_SECTORANGLE = 360.0 / PC_NUM_SECTOR;
  sc_manager.PC_UNIT_RINGGAP = PC_MAX_RADIUS / PC_NUM_RING;

  const double db_radius = PC_MAX_RADIUS + 2.0;
  struct Entry { double x, y; Eigen::MatrixXd desc, key; };
  std::vector<Entry> entries;
  int skipped = 0;
  for (double cx = min_x; cx <= max_x; cx += grid_step) {
    for (double cy = min_y; cy <= max_y; cy += grid_step) {
      auto crop = crop_lidar_frame(map, cx, cy, db_radius, lidar_height);
      crop = voxel(crop, 0.3);
      if (crop->size() < 200) { ++skipped; continue; }
      Entry e;
      e.x = cx; e.y = cy;
      e.desc = make_surface_desc(*crop);  // surface-band encoding (v2)
      e.key = sc_manager.makeRingkeyFromScancontext(e.desc);
      entries.push_back(std::move(e));
    }
  }
  std::cout << "[scdb] entries=" << entries.size() << " skipped=" << skipped << std::endl;

  FILE* f = std::fopen(out_path.c_str(), "wb");
  if (!f) { std::cerr << "[scdb] cannot open " << out_path << std::endl; return 1; }
  const char magic[5] = {'S', 'C', 'D', 'B', '1'};
  const uint32_t version = 3; // v3: surface-band encoding, no FOV band (elevation-safe)
  const uint32_t count = entries.size();
  const float step = static_cast<float>(grid_step);
  std::fwrite(magic, 1, 5, f);
  std::fwrite(&version, sizeof(version), 1, f);
  std::fwrite(&step, sizeof(step), 1, f);
  std::fwrite(&count, sizeof(count), 1, f);
  for (const auto& e : entries) {
    const float x = static_cast<float>(e.x), y = static_cast<float>(e.y);
    std::fwrite(&x, sizeof(x), 1, f);
    std::fwrite(&y, sizeof(y), 1, f);
    for (int r = 0; r < PC_NUM_RING; ++r)
      for (int s = 0; s < PC_NUM_SECTOR; ++s) {
        const float v = static_cast<float>(e.desc(r, s));
        std::fwrite(&v, sizeof(v), 1, f);
      }
    for (int r = 0; r < PC_NUM_RING; ++r) {
      const float v = static_cast<float>(e.key(r, 0));
      std::fwrite(&v, sizeof(v), 1, f);
    }
  }
  std::fclose(f);
  std::cout << "[scdb] wrote " << out_path << " (" << count << " entries, ~"
            << (count * (PC_NUM_RING * PC_NUM_SECTOR + PC_NUM_RING + 2) * 4 / 1024 / 1024)
            << " MB descriptors)" << std::endl;
  return 0;
}
