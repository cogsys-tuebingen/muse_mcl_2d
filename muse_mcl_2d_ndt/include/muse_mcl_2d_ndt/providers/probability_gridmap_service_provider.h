#ifndef MUSE_MCL_2D_NDT_PROBABILITY_GRIDMAP_SERVICE_PROVIDER_H
#define MUSE_MCL_2D_NDT_PROBABILITY_GRIDMAP_SERVICE_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/probability_gridmap.h>

namespace muse_mcl_2d_ndt {
class ProbabilityGridmapServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    ProbabilityGridmapServiceProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient                              source_;
    std::string                                             service_name_;
    std::string                                             path_;
    std::string                                             frame_id_;
    double                                                  sampling_resolution_;

    mutable std::mutex                                      map_mutex_;
    mutable std::condition_variable                         map_notify_;
    mutable muse_mcl_2d_gridmaps::ProbabilityGridmap::Ptr   map_;
    mutable std::thread                                     worker_;

    void loadMap() const;
};
}

#endif // MUSE_MCL_2D_NDT_PROBABILITY_GRIDMAP_SERVICE_PROVIDER_H
