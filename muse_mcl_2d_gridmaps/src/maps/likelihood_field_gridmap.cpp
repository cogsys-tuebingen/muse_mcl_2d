#include <muse_mcl_2d_gridmaps/maps/likelihood_field_gridmap.h>


namespace muse_mcl_2d_gridmaps {
LikelihoodFieldGridmap::LikelihoodFieldGridmap(const cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr &map,
                                               const std::string frame_id) :
     muse_mcl_2d::Map2D(frame_id),
     data_(map)
{
}

LikelihoodFieldGridmap::state_space_boundary_t LikelihoodFieldGridmap::getMin() const
{
    return data_->getMin();
}

LikelihoodFieldGridmap::state_space_boundary_t LikelihoodFieldGridmap::getMax() const
{
    return data_->getMax();
}

LikelihoodFieldGridmap::state_space_transform_t LikelihoodFieldGridmap::getOrigin() const
{
    return data_->getOrigin();
}

bool LikelihoodFieldGridmap::validate(const cslibs_math_2d::Pose2d &p_w) const
{
    return data_->validate(p_w);
}

cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr& LikelihoodFieldGridmap::data()
{
    return data_;
}

cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr const & LikelihoodFieldGridmap::data() const
{
    return data_;
}



}
