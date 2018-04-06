#include <muse_mcl_2d_gridmaps/maps/binary_gridmap.h>

namespace muse_mcl_2d_gridmaps {
BinaryGridmap::BinaryGridmap(const cslibs_gridmaps::static_maps::BinaryGridmap::Ptr &map,
                             const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

BinaryGridmap::state_space_boundary_t BinaryGridmap::getMin() const
{
    return data_->getMin();
}

BinaryGridmap::state_space_boundary_t BinaryGridmap::getMax() const
{
    return data_->getMax();
}

BinaryGridmap::state_space_transform_t BinaryGridmap::getOrigin() const
{
    return data_->getOrigin();
}

bool BinaryGridmap::validate(const cslibs_math_2d::Pose2d &p) const
{
    return data_->validate(p);
}

cslibs_gridmaps::static_maps::BinaryGridmap::Ptr& BinaryGridmap::data()
{
    return data_;
}

cslibs_gridmaps::static_maps::BinaryGridmap::Ptr const & BinaryGridmap::data() const
{
    return data_;
}
}

