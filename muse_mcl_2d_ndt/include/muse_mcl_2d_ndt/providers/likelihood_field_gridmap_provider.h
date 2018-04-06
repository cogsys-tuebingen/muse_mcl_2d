#ifndef MUSE_MCL_2D_NDT_LIKELIHOOD_FIELD_GRIDMAP_PROVIDER_H
#define MUSE_MCL_2D_NDT_LIKELIHOOD_FIELD_GRIDMAP_PROVIDER_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/likelihood_field_gridmap.h>

namespace muse_mcl_2d_ndt {
class LikelihoodFieldGridmapProvider : public muse_mcl_2d::MapProvider2D
{
public:
    LikelihoodFieldGridmapProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                                         path_;
    std::string                                         frame_id_;

    double                                              sampling_resolution_;
    double                                              maximum_distance_;
    double                                              sigma_hit_;
    double                                              threshold_;

    mutable std::mutex                                  map_mutex_;
    mutable std::condition_variable                     map_notify_;
    muse_mcl_2d_gridmaps::LikelihoodFieldGridmap::Ptr   map_;
    std::thread                                         worker_;

    void loadMap();
};
}

#endif // MUSE_MCL_2D_NDT_LIKELIHOOD_FIELD_GRIDMAP_PROVIDER_H
