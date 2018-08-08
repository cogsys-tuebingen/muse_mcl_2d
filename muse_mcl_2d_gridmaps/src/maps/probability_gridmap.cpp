#include <muse_mcl_2d_gridmaps/maps/probability_gridmap.h>

namespace muse_mcl_2d_gridmaps {
ProbabilityGridmap::ProbabilityGridmap(const cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr &map,
                                       const std::string frame_id) :
     muse_mcl_2d::Map2D(frame_id),
     data_(map)
{
}

ProbabilityGridmap::state_space_boundary_t ProbabilityGridmap::getMin() const
{
    return data_->getOrigin() * data_->getMin();
}

ProbabilityGridmap::state_space_boundary_t ProbabilityGridmap::getMax() const
{
    return data_->getOrigin() * data_->getMax();
}

ProbabilityGridmap::state_space_transform_t ProbabilityGridmap::getOrigin() const
{
    return data_->getOrigin();
}

bool ProbabilityGridmap::validate(const cslibs_math_2d::Pose2d &p_w) const
{
    return true; //data_->validate(p_w);
}

cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr& ProbabilityGridmap::data()
{
    return data_;
}

cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr const & ProbabilityGridmap::data() const
{
    return data_;
}
}
