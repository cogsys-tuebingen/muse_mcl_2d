#ifndef MUSE_MCL_2D_NDT_LIKELIHOOD_FIELD_OCCUPANCY_GRIDMAP_SERVICE_PROVIDER_H
#define MUSE_MCL_2D_NDT_LIKELIHOOD_FIELD_OCCUPANCY_GRIDMAP_SERVICE_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/likelihood_field_gridmap.h>

#include <cslibs_gridmaps/utility/inverse_model.hpp>

namespace muse_mcl_2d_ndt {
class LikelihoodFieldOccupancyGridmapServiceProvider : public muse_mcl_2d::MapProvider2D
{
public:
    LikelihoodFieldOccupancyGridmapServiceProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    mutable ros::ServiceClient                                  source_;
    std::string                                                 service_name_;
    std::string                                                 path_;
    std::string                                                 frame_id_;

    double                                                      sampling_resolution_;
    double                                                      maximum_distance_;
    double                                                      sigma_hit_;
    double                                                      threshold_;
    cslibs_gridmaps::utility::InverseModel::Ptr                 inverse_model_;

    mutable std::mutex                                          map_mutex_;
    mutable std::condition_variable                             map_notify_;
    mutable muse_mcl_2d_gridmaps::LikelihoodFieldGridmap::Ptr   map_;
    mutable std::thread                                         worker_;

    void loadMap() const;
};
}

#endif // MUSE_MCL_2D_NDT_LIKELIHOOD_FIELD_OCCUPANCY_GRIDMAP_SERVICE_PROVIDER_H
