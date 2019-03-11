#include <muse_mcl_2d_gridmaps/maps/likelihood_field_gridmap.h>


namespace muse_mcl_2d_gridmaps {
LikelihoodFieldGridmap::LikelihoodFieldGridmap(const map_t::Ptr &map,
                                               const std::string frame_id) :
     muse_mcl_2d::Map2D(frame_id),
     data_(map)
{
}

LikelihoodFieldGridmap::state_space_boundary_t LikelihoodFieldGridmap::getMin() const
{
    return data_->getOrigin() * data_->getMin();
}

LikelihoodFieldGridmap::state_space_boundary_t LikelihoodFieldGridmap::getMax() const
{
    return data_->getOrigin() * data_->getMax();
}

LikelihoodFieldGridmap::state_space_transform_t LikelihoodFieldGridmap::getOrigin() const
{
    return data_->getOrigin();
}

bool LikelihoodFieldGridmap::validate(const state_t &p_w) const
{
    return data_->validate(p_w);
}

LikelihoodFieldGridmap::map_t::Ptr& LikelihoodFieldGridmap::data()
{
    return data_;
}

LikelihoodFieldGridmap::map_t::Ptr const & LikelihoodFieldGridmap::data() const
{
    return data_;
}
}
