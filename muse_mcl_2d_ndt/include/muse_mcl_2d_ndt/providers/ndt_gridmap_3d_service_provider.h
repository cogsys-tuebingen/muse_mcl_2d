#ifndef MUSE_MCL_2D_NDT_GRIDMAP_3D_SERVICE_PROVIDER_H
#define MUSE_MCL_2D_NDT_GRIDMAP_3D_SERVICE_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

namespace muse_mcl_2d_ndt {
class NDTGridmap3dServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    NDTGridmap3dServiceProvider() = default;
    virtual ~NDTGridmap3dServiceProvider() = default;

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient      source_;
    std::string                     service_name_;

    mutable Gridmap3d::Ptr          map_;
};
}

#endif // MUSE_MCL_2D_NDT_GRIDMAP_3D_SERVICE_PROVIDER_H
