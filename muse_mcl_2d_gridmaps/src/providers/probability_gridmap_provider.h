#ifndef DATA_PROVIDER_PROBABILITY_GRIDMAP_H
#define DATA_PROVIDER_PROBABILITY_GRIDMAP_H

#include <nav_msgs/OccupancyGrid.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/probability_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class ProbabilityGridmapProvider : public muse_mcl_2d::MapProvider2D
{
public:
    ProbabilityGridmapProvider() = default;
    virtual ~ProbabilityGridmapProvider() = default;

    using base_t = muse_mcl_2d::MapProvider2D;
    using base_t::state_space_t;

    std::shared_ptr<state_space_t const> getStateSpace() const override;
    void waitForStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    ros::Subscriber source_;
    std::string     topic_;

    mutable std::mutex                                  map_mutex_;
    muse_mcl_2d_gridmaps::ProbabilityGridmap::Ptr       map_;
    std::thread                                         worker_;
    mutable std::condition_variable                     notify_;

    void callback(const nav_msgs::OccupancyGridConstPtr &msg);


};
}

#endif // DATA_PROVIDER_PROBABILITY_GRIDMAP_H
