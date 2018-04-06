#ifndef MUSE_MCL_2D_NDT_GRIDMAP_2D_SERVICE_PROVIDER_H
#define MUSE_MCL_2D_NDT_GRIDMAP_2D_SERVICE_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap_2d.h>

namespace muse_mcl_2d_ndt {
class NDTGridmap2dServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    NDTGridmap2dServiceProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient      source_;
    std::string                     service_name_;
    std::string                     path_;
    std::string                     frame_id_;

    mutable std::mutex              map_mutex_;
    mutable std::condition_variable map_notify_;
    mutable Gridmap2d::Ptr          map_;
    mutable std::thread             worker_;

    void loadMap() const;
};
}

#endif // MUSE_MCL_2D_NDT_GRIDMAP_2D_SERVICE_PROVIDER_H
