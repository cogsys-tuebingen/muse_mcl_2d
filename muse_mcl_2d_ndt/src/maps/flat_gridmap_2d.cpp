#include <muse_mcl_2d_ndt/maps/flat_gridmap_2d.h>

namespace muse_mcl_2d_ndt {
FlatGridmap2D::FlatGridmap2D(const map_t::Ptr &map,
                 const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

FlatGridmap2D::state_space_boundary_t FlatGridmap2D::getMin() const
{
    return data_->getInitialOrigin() * data_->getMin();
}

FlatGridmap2D::state_space_boundary_t FlatGridmap2D::getMax() const
{
    return data_->getInitialOrigin() * data_->getMax();
}

FlatGridmap2D::state_space_transform_t FlatGridmap2D::getOrigin() const
{
    return data_->getOrigin();
}

bool FlatGridmap2D::validate(const state_t &p) const
{
    return data_->validate(p);
}

FlatGridmap2D::map_t::Ptr& FlatGridmap2D::data()
{
    return data_;
}

FlatGridmap2D::map_t::Ptr const& FlatGridmap2D::data() const
{
    return data_;
}
}
