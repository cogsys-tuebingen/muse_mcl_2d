#ifndef DATA_PROVIDER_DISTANCE_GRIDMAP_H
#define DATA_PROVIDER_DISTANCE_GRIDMAP_H

#include <nav_msgs/OccupancyGrid.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/distance_gridmap.h>


namespace muse_mcl_2d_gridmaps {
class DistanceGridmapProvider : public muse_mcl_2d::MapProvider2D
{
public:
    DistanceGridmapProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    ros::Subscriber                     source_;
    std::string                         topic_;
    double                              binarization_threshold_;
    double                              maximum_distance_;
    bool                                blocking_;

    mutable std::mutex                  map_mutex_;
    DistanceGridmap::Ptr                map_;
    mutable std::mutex                  map_load_mutex_;
    std::thread                         worker_;
    mutable std::condition_variable     notify_;

    void callback(const nav_msgs::OccupancyGridConstPtr &msg);


};
}

#endif // DATA_PROVIDER_DISTANCE_GRIDMAP_H
