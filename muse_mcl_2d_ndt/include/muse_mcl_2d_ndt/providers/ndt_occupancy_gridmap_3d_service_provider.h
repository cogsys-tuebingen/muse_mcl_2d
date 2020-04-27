#ifndef MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_SERVICE_PROVIDER_H
#define MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_SERVICE_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>

#include <sensor_msgs/PointCloud2.h>

namespace muse_mcl_2d_ndt {
class NDTOccupancyGridmap3dServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    NDTOccupancyGridmap3dServiceProvider() = default;
    virtual ~NDTOccupancyGridmap3dServiceProvider() = default;

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient      source_;
    std::string                     service_name_;
    mutable OccupancyGridmap3d::Ptr map_;
};
}

#endif // MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_SERVICE_PROVIDER_H
