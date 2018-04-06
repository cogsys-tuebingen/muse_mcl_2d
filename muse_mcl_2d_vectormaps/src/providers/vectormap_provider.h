#ifndef DATA_PROVIDER_VECTORMAP_H
#define DATA_PROVIDER_VECTORMAP_H

#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d_vectormaps/static_maps/vectormap.h>

namespace muse_mcl_2d_vectormaps {

class VectorMapProvider : public muse_mcl_2d::MapProvider2D {
public:
    VectorMapProvider();

    state_space_t::ConstPtr getStateSpace() const override;
    void setup(ros::NodeHandle &nh) override;

protected:
    static_maps::VectorMap::Ptr map_;
};

}

#endif // DATA_PROVIDER_VECTORMAP_H
