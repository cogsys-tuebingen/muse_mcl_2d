#ifndef MUSE_MCL_2D_NDT_PROBABILITY_GRIDMAP_PROVIDER_H
#define MUSE_MCL_2D_NDT_PROBABILITY_GRIDMAP_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/probability_gridmap.h>

namespace muse_mcl_2d_ndt {
class ProbabilityGridmapProvider : public muse_mcl_2d::MapProvider2D
{
public:
    ProbabilityGridmapProvider() = default;
    virtual ~ProbabilityGridmapProvider() = default;

    std::shared_ptr<state_space_t const> getStateSpace() const override;
    void waitForStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                                     path_;
    std::string                                     frame_id_;
    double                                          sampling_resolution_;

    mutable std::mutex                              map_mutex_;
    mutable std::condition_variable                 map_notify_;
    muse_mcl_2d_gridmaps::ProbabilityGridmap::Ptr   map_;
    std::thread                                     worker_;
};
}

#endif // MUSE_MCL_2D_NDT_PROBABILITY_GRIDMAP_PROVIDER_H
