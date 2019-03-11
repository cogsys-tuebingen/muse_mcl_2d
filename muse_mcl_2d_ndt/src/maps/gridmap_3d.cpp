#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

namespace muse_mcl_2d_ndt {
Gridmap3d::Gridmap3d(const map_t::Ptr &map,
                     const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

Gridmap3d::state_space_boundary_t Gridmap3d::getMin() const
{
    const typename map_t::point_t &min = data_->getInitialOrigin() * data_->getMin();
    return point_t(min(0), min(1));
}

Gridmap3d::state_space_boundary_t Gridmap3d::getMax() const
{
    const typename map_t::point_t &max = data_->getInitialOrigin() * data_->getMax();
    return point_t(max(0), max(1));
}

Gridmap3d::state_space_transform_t Gridmap3d::getOrigin() const
{
    const typename map_t::pose_t &origin = data_->getInitialOrigin();
    return state_t(origin.tx(), origin.ty(), origin.yaw());
}

bool Gridmap3d::validate(const state_t &p) const
{
  return data_->validate(p);
}

Gridmap3d::map_t::Ptr& Gridmap3d::data()
{
    return data_;
}

Gridmap3d::map_t::Ptr const& Gridmap3d::data() const
{
    return data_;
}
}
