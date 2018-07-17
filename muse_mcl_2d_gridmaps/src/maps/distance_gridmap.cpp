#include <muse_mcl_2d_gridmaps/maps/distance_gridmap.h>

namespace muse_mcl_2d_gridmaps {
DistanceGridmap::DistanceGridmap(const cslibs_gridmaps::static_maps::DistanceGridmap::Ptr &map,
                                 const std::string frame_id) :
     muse_mcl_2d::Map2D(frame_id),
     data_(map)
{
}

DistanceGridmap::state_space_boundary_t DistanceGridmap::getMin() const
{
    return data_->getMin();
}

DistanceGridmap::state_space_boundary_t DistanceGridmap::getMax() const
{
    return data_->getMax();
}

DistanceGridmap::state_space_transform_t DistanceGridmap::getOrigin() const
{
    return data_->getOrigin();
}

bool DistanceGridmap::validate(const cslibs_math_2d::Pose2d &p_w) const
{
    return data_->validate(p_w);
}

cslibs_gridmaps::static_maps::DistanceGridmap::Ptr& DistanceGridmap::data()
{
    return data_;
}

cslibs_gridmaps::static_maps::DistanceGridmap::Ptr const & DistanceGridmap::data() const
{
    return data_;
}
}
