#include <muse_mcl_2d_ndt/maps/gridmap_2d.h>

namespace muse_mcl_2d_ndt {
Gridmap2d::Gridmap2d(const cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr &map,
                 const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

Gridmap2d::state_space_boundary_t Gridmap2d::getMin() const
{
    return data_->getInitialOrigin() * data_->getMin();
}

Gridmap2d::state_space_boundary_t Gridmap2d::getMax() const
{
    return data_->getInitialOrigin() * data_->getMax();
}

Gridmap2d::state_space_transform_t Gridmap2d::getOrigin() const
{
    return data_->getInitialOrigin();
}

bool Gridmap2d::validate(const cslibs_math_2d::Pose2d &p) const
{
  return data_->validate(p);
}

cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr& Gridmap2d::data()
{
    return data_;
}

cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr const& Gridmap2d::data() const
{
    return data_;
}
}
