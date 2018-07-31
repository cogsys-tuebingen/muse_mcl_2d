#ifndef DATA_PROVIDER_DISTANCE_GRIDMAP_LOAD_H
#define DATA_PROVIDER_DISTANCE_GRIDMAP_LOAD_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/distance_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class DistanceGridmapLoadProvider : public muse_mcl_2d::MapProvider2D
{
public:
    DistanceGridmapLoadProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    double                                              binarization_threshold_;
    double                                              maximum_distance_;
    bool                                                blocking_;

    mutable std::mutex                                  map_mutex_;
    muse_mcl_2d_gridmaps::DistanceGridmap::Ptr          map_;
    mutable std::mutex                                  map_load_mutex_;
    std::thread                                         worker_;
    mutable std::condition_variable                     notify_;
};
}

#endif // DATA_PROVIDER_DISTANCE_GRIDMAP_LOAD_H
