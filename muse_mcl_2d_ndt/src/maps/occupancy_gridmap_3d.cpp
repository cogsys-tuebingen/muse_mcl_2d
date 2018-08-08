#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>

namespace muse_mcl_2d_ndt {
OccupancyGridmap3d::OccupancyGridmap3d(const cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::Ptr &map,
                 const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

OccupancyGridmap3d::state_space_boundary_t OccupancyGridmap3d::getMin() const
{
    const typename cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::point_t &min = data_->getInitialOrigin() * data_->getMin();
    return cslibs_math_2d::Point2d(min(0), min(1));
}

OccupancyGridmap3d::state_space_boundary_t OccupancyGridmap3d::getMax() const
{
    const typename cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::point_t &max = data_->getInitialOrigin() * data_->getMax();
    return cslibs_math_2d::Point2d(max(0), max(1));
}

OccupancyGridmap3d::state_space_transform_t OccupancyGridmap3d::getOrigin() const
{
    const typename cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::pose_t &origin = data_->getInitialOrigin();
    return cslibs_math_2d::Pose2d(origin.tx(), origin.ty(), origin.yaw());
}

bool OccupancyGridmap3d::validate(const cslibs_math_2d::Pose2d &p) const
{
  return data_->validate(p);
}

cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::Ptr& OccupancyGridmap3d::data()
{
    return data_;
}

cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::Ptr const& OccupancyGridmap3d::data() const
{
    return data_;
}
}
