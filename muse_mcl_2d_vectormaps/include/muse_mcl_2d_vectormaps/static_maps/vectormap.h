#ifndef VECTORMAP_H
#define VECTORMAP_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_vectormaps/maps/vector_map.h>

namespace muse_mcl_2d_vectormaps {
namespace static_maps {

class VectorMap : public muse_mcl_2d::Map2D {
public:
    using Ptr = std::shared_ptr<VectorMap>;

    VectorMap(cslibs_vectormaps::VectorMap::Ptr vector_map) :
        muse_mcl_2d::Map2D("map"),
        vector_map_(vector_map)
    {
    }

    virtual cslibs_math_2d::Point2d getMin() const override
    {
        cslibs_vectormaps::VectorMap::Point p = vector_map_->minCorner();
        return cslibs_math_2d::Point2d(p.x(), p.y());
    }

    virtual cslibs_math_2d::Point2d getMax() const override
    {
        cslibs_vectormaps::VectorMap::Point p = vector_map_->maxCorner();
        return cslibs_math_2d::Point2d(p.x(), p.y());
    }

    virtual cslibs_math_2d::Pose2d getOrigin() const override
    {
        cslibs_math_2d::Pose2d origin = cslibs_math_2d::Pose2d::identity();
        origin.translation() = getMin();
        return origin;
    }

    virtual bool validate(const cslibs_math_2d::Pose2d &) const override
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
