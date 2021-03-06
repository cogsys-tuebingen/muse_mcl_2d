#ifndef MAP_PROVIDER_BINARY_GRIDMAP_SERVICE_H
#define MAP_PROVIDER_BINARY_GRIDMAP_SERVICE_H

#include <nav_msgs/OccupancyGrid.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/binary_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class BinaryGridmapServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    BinaryGridmapServiceProvider() = default;
    virtual ~BinaryGridmapServiceProvider() = default;

    using base_t = muse_mcl_2d::MapProvider2D;
    using base_t::state_space_t;

    std::shared_ptr<state_space_t const> getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient                          source_;
    std::string                                         service_name_;
    double                                              binarization_threshold_;

    mutable muse_mcl_2d_gridmaps::BinaryGridmap::Ptr    map_;

};
}


#endif // MAP_PROVIDER_BINARY_GRIDMAP_SERVICE_H
