#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_2d.h>

namespace muse_mcl_2d_ndt {
OccupancyGridmap2d::OccupancyGridmap2d(const map_t::Ptr &map,
                                       const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

OccupancyGridmap2d::state_space_boundary_t OccupancyGridmap2d::getMin() const
{
    return data_->getInitialOrigin() * data_->getMin();
}

OccupancyGridmap2d::state_space_boundary_t OccupancyGridmap2d::getMax() const
{
    return data_->getInitialOrigin() * data_->getMax();
}

OccupancyGridmap2d::state_space_transform_t OccupancyGridmap2d::getOrigin() const
{
    return data_->getInitialOrigin();
}

bool OccupancyGridmap2d::validate(const state_t &p) const
{
  return data_->validate(p);
}

OccupancyGridmap2d::map_t::Ptr& OccupancyGridmap2d::data()
{
    return data_;
}

OccupancyGridmap2d::map_t::Ptr const& OccupancyGridmap2d::data() const
{
    return data_;
}
}
