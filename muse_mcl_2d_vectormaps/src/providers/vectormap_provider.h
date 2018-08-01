#ifndef DATA_PROVIDER_VECTORMAP_H
#define DATA_PROVIDER_VECTORMAP_H

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_vectormaps/static_maps/vectormap.h>
#include <mutex>
#include <condition_variable>

namespace muse_mcl_2d_vectormaps {
class VectorMapProvider : public muse_mcl_2d::MapProvider2D {
public:
    VectorMapProvider() = default;
    virtual ~VectorMapProvider() = default;

    state_space_t::ConstPtr getStateSpace() const override;
    void waitForStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    std::string                     map_file_;

    mutable std::mutex              map_mutex_;
    static_maps::VectorMap::Ptr     map_;
    std::thread                     worker_;
    mutable std::condition_variable notify_;
};
}

#endif // DATA_PROVIDER_VECTORMAP_H
