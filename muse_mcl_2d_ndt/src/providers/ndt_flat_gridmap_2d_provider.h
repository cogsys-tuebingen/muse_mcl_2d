#ifndef NDT_FLAT_GRIDMAP_2D_PROVIDER_H
#define NDT_FLAT_GRIDMAP_2D_PROVIDER_H


#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_ndt/maps/flat_gridmap_2d.h>

namespace muse_mcl_2d_ndt {
class NDTFlatGridmap2DProvider : public muse_mcl_2d::MapProvider2D
{
public:
    NDTFlatGridmap2DProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                     path_;
    std::string                     frame_id_;

    mutable std::mutex              map_mutex_;
    mutable std::condition_variable map_notify_;
    FlatGridmap2D::Ptr              map_;
    std::thread                     worker_;

    void loadMap();
};
}

#endif // NDT_FLAT_GRIDMAP_2D_PROVIDER_H
