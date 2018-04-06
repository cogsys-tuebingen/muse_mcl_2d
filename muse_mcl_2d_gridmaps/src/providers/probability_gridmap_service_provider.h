#ifndef MAP_PROVIDER_PROBABILITY_GRIDMAP_SERVICE_H
#define MAP_PROVIDER_PROBABILITY_GRIDMAP_SERVICE_H

#include <nav_msgs/GetMap.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/probability_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class ProbabilityGridmapServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    ProbabilityGridmapServiceProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient                              source_;
    std::string                                             service_name_;
    bool                                                    blocking_;

    mutable std::mutex                                      map_mutex_;
    mutable muse_mcl_2d_gridmaps::ProbabilityGridmap::Ptr   map_;
    mutable std::mutex                                      map_load_mutex_;
    mutable std::thread                                     worker_;
    mutable std::condition_variable                         notify_;

};
}


#endif // MAP_PROVIDER_PROBABILITY_GRIDMAP_SERVICE_H
