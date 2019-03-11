#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>

namespace muse_mcl_2d_ndt {
OccupancyGridmap3d::OccupancyGridmap3d(const map_t::Ptr &map,
                                       const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

OccupancyGridmap3d::state_space_boundary_t OccupancyGridmap3d::getMin() const
{
    const typename map_t::point_t &min = data_->getInitialOrigin() * data_->getMin();
    return point_t(min(0), min(1));
}

OccupancyGridmap3d::state_space_boundary_t OccupancyGridmap3d::getMax() const
{
    const typename map_t::point_t &max = data_->getInitialOrigin() * data_->getMax();
    return point_t(max(0), max(1));
}

OccupancyGridmap3d::state_space_transform_t OccupancyGridmap3d::getOrigin() const
{
    const typename map_t::pose_t &origin = data_->getInitialOrigin();
    return state_t(origin.tx(), origin.ty(), origin.yaw());
}

bool OccupancyGridmap3d::validate(const state_t &p) const
{
  return data_->validate(p);
}

OccupancyGridmap3d::map_t::Ptr& OccupancyGridmap3d::data()
{
    return data_;
}

OccupancyGridmap3d::map_t::Ptr const& OccupancyGridmap3d::data() const
{
    return data_;
}
}
