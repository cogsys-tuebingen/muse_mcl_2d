#ifndef ORIENTED_GRID_VECTORMAP_PROVIDER_H
#define ORIENTED_GRID_VECTORMAP_PROVIDER_H

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_vectormaps/static_maps/oriented_grid_vectormap.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>
#include <mutex>
#include <condition_variable>

namespace muse_mcl_2d_vectormaps {
class OrientedGridVectorMapProvider : public muse_mcl_2d::MapProvider2D {
public:
    OrientedGridVectorMapProvider() = default;
    virtual ~OrientedGridVectorMapProvider() = default;

    state_space_t::ConstPtr getStateSpace() const override;
    void waitForStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                             map_file_;

    mutable std::mutex                      map_mutex_;
    static_maps::OrientedGridVectorMap::Ptr map_;
    std::thread                             worker_;
    mutable std::condition_variable         notify_;
};
}

#endif // ORIENTED_GRID_VECTORMAP_PROVIDER_H
