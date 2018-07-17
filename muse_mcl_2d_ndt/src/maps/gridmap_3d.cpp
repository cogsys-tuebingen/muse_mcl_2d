#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

namespace muse_mcl_2d_ndt {
Gridmap3d::Gridmap3d(const cslibs_ndt_3d::dynamic_maps::Gridmap::Ptr &map,
                     const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

Gridmap3d::state_space_boundary_t Gridmap3d::getMin() const
{
    const typename cslibs_ndt_3d::dynamic_maps::Gridmap::point_t &min = data_->getMin();
    return cslibs_math_2d::Point2d(min(0), min(1));
}

Gridmap3d::state_space_boundary_t Gridmap3d::getMax() const
{
    const typename cslibs_ndt_3d::dynamic_maps::Gridmap::point_t &max = data_->getMax();
    return cslibs_math_2d::Point2d(max(0), max(1));
}

Gridmap3d::state_space_transform_t Gridmap3d::getOrigin() const
{
    const typename cslibs_ndt_3d::dynamic_maps::Gridmap::pose_t &origin = data_->getInitialOrigin();
    return cslibs_math_2d::Pose2d(origin.tx(), origin.ty(), origin.yaw());
}

bool Gridmap3d::validate(const cslibs_math_2d::Pose2d &p) const
{
  return data_->validate(p);
}

cslibs_ndt_3d::dynamic_maps::Gridmap::Ptr& Gridmap3d::data()
{
    return data_;
}

cslibs_ndt_3d::dynamic_maps::Gridmap::Ptr const& Gridmap3d::data() const
{
    return data_;
}
}
