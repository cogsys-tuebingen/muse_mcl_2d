#ifndef MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_PROVIDER_H
#define MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_PROVIDER_H

#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>

#include <atomic>
#include <condition_variable>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <mutex>
#include <thread>

namespace muse_mcl_2d_ndt {
class NDTOccupancyGridmap3dProvider : public muse_mcl_2d::MapProvider2D {
 public:
  NDTOccupancyGridmap3dProvider() = default;
  virtual ~NDTOccupancyGridmap3dProvider() = default;

  using base_t = muse_mcl_2d::MapProvider2D;
  using base_t::state_space_t;

  std::shared_ptr<state_space_t const> getStateSpace() const override;
  void waitForStateSpace() const override;
  void setup(ros::NodeHandle &nh) override;

 protected:
  std::string path_;
  std::string frame_id_;

  mutable std::mutex map_mutex_;
  mutable std::condition_variable map_notify_;
  OccupancyGridmap3d::Ptr map_;
  std::thread worker_;
};
}  // namespace muse_mcl_2d_ndt

#endif  // MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_PROVIDER_H
