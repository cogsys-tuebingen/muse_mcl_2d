#ifndef MAP_PROVIDER_DISTANCE_GRIDMAP_SERVICE_H
#define MAP_PROVIDER_DISTANCE_GRIDMAP_SERVICE_H

#include <nav_msgs/GetMap.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/distance_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class DistanceGridmapServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    DistanceGridmapServiceProvider() = default;
    virtual ~DistanceGridmapServiceProvider() = default;

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient                  source_;
    std::string                                 service_name_;
    double                                      binarization_threshold_;
    double                                      maximum_distance_;

    mutable DistanceGridmap::Ptr                map_;

};
}

#endif // MAP_PROVIDER_DISTANCE_GRIDMAP_SERVICE_H
