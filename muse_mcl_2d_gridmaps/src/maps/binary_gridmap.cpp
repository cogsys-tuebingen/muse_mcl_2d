#include <muse_mcl_2d_gridmaps/maps/binary_gridmap.h>

namespace muse_mcl_2d_gridmaps {
BinaryGridmap::BinaryGridmap(const map_t::Ptr &map,
                             const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

BinaryGridmap::state_space_boundary_t BinaryGridmap::getMin() const
{
    return data_->getOrigin() * data_->getMin();
}

BinaryGridmap::state_space_boundary_t BinaryGridmap::getMax() const
{
    return data_->getOrigin() * data_->getMax();
}

BinaryGridmap::state_space_transform_t BinaryGridmap::getOrigin() const
{
    return data_->getOrigin();
}

bool BinaryGridmap::validate(const state_t &p) const
{
    return data_->validate(p);
}

BinaryGridmap::map_t::Ptr& BinaryGridmap::data()
{
    return data_;
}

BinaryGridmap::map_t::Ptr const & BinaryGridmap::data() const
{
    return data_;
}
}

