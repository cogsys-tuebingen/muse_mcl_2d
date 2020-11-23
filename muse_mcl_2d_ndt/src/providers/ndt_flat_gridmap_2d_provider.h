#ifndef NDT_FLAT_GRIDMAP_2D_PROVIDER_H
#define NDT_FLAT_GRIDMAP_2D_PROVIDER_H

#include <muse_mcl_2d_ndt/maps/flat_gridmap_2d.h>

#include <atomic>
#include <condition_variable>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <mutex>
#include <thread>

namespace muse_mcl_2d_ndt {
class NDTFlatGridmap2DProvider : public muse_mcl_2d::MapProvider2D {
 public:
  NDTFlatGridmap2DProvider();

  using base_t = muse_mcl_2d::MapProvider2D;
  using base_t::state_space_t;

  std::shared_ptr<state_space_t const> getStateSpace() const override;
  void setup(ros::NodeHandle &nh) override;

 protected:
  std::string path_;
  std::string frame_id_;

  mutable std::mutex map_mutex_;
  mutable std::condition_variable map_notify_;
  FlatGridmap2D::Ptr map_;
  std::thread worker_;

  void loadMap();
};
}  // namespace muse_mcl_2d_ndt

#endif  // NDT_FLAT_GRIDMAP_2D_PROVIDER_H
