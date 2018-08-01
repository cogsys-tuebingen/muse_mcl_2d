#ifndef DATA_PROVIDER_PROBABILITY_GRIDMAP_LOAD_H
#define DATA_PROVIDER_PROBABILITY_GRIDMAP_LOAD_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/probability_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class ProbabilityGridmapLoadProvider : public muse_mcl_2d::MapProvider2D
{
public:
    ProbabilityGridmapLoadProvider() = default;
    virtual ~ProbabilityGridmapLoadProvider() = default;

    state_space_t::ConstPtr getStateSpace() const override;
    void waitForStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable std::mutex                                  map_mutex_;
    muse_mcl_2d_gridmaps::ProbabilityGridmap::Ptr       map_;
    std::thread                                         worker_;
    mutable std::condition_variable                     notify_;
};
}

#endif // DATA_PROVIDER_PROBABILITY_GRIDMAP_LOAD_H
