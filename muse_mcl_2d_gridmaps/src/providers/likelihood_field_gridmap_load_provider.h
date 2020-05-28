#ifndef DATA_PROVIDER_LIKELIHOOD_FIELD_GRIDMAP_LOAD_H
#define DATA_PROVIDER_LIKELIHOOD_FIELD_GRIDMAP_LOAD_H

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_gridmaps/maps/likelihood_field_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class LikelihoodFieldGridmapLoadProvider : public muse_mcl_2d::MapProvider2D
{
public:
    LikelihoodFieldGridmapLoadProvider() = default;
    virtual ~LikelihoodFieldGridmapLoadProvider() = default;

    using base_t = muse_mcl_2d::MapProvider2D;
    using base_t::state_space_t;

    std::shared_ptr<state_space_t const> getStateSpace() const override;
    void waitForStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    double                                              binarization_threshold_;
    double                                              maximum_distance_;
    double                                              z_hit_;
    double                                              sigma_hit_;

    mutable std::mutex                                  map_mutex_;
    muse_mcl_2d_gridmaps::LikelihoodFieldGridmap::Ptr   map_;
    std::thread                                         worker_;
    mutable std::condition_variable                     notify_;
};
}

#endif // DATA_PROVIDER_LIKELIHOOD_FIELD_GRIDMAP_LOAD_H
