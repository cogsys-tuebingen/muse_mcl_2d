#ifndef VECTORMAP_H
#define VECTORMAP_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_vectormaps/maps/vector_map.h>

namespace muse_mcl_2d_vectormaps {
namespace static_maps {
class VectorMap : public muse_mcl_2d::Map2D {
public:
    using Ptr = std::shared_ptr<VectorMap>;
    using point_t = muse_mcl_2d::Sample2D::state_space_boundary_t;
    using state_t = muse_mcl_2d::Sample2D::state_t;

    VectorMap(cslibs_vectormaps::VectorMap::Ptr vector_map) :
        muse_mcl_2d::Map2D("map"),
        vector_map_(vector_map)
    {
    }

    virtual point_t getMin() const override
    {
        cslibs_vectormaps::VectorMap::Point p = vector_map_->minCorner();
        return point_t(p.x(), p.y());
    }

    virtual point_t getMax() const override
    {
        cslibs_vectormaps::VectorMap::Point p = vector_map_->maxCorner();
        return point_t(p.x(), p.y());
    }

    virtual state_t getOrigin() const override
    {
        state_t origin = state_t::identity();
        origin.translation() = getMin();
        return origin;
    }

    virtual bool validate(const state_t &) const override
    {
        return true;
    }

    cslibs_vectormaps::VectorMap &getMap() const
    {
        return *vector_map_;
    }

protected:
    cslibs_vectormaps::VectorMap::Ptr vector_map_;
};

}
}

#endif // VECTOR_MAP_H
