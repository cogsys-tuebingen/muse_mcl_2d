#ifndef LIKELIHOOD_FIELD_MODEL_VECTOR_H
#define LIKELIHOOD_FIELD_MODEL_VECTOR_H

#include <muse_mcl_2d/update/update_model_2d.hpp>

namespace muse_mcl_2d_vectormaps {
class LikelihoodFieldModelVector :  public muse_mcl_2d::UpdateModel2D {
public:
    using transform_t = muse_mcl_2d::StateSpaceDescription2D::transform_t;
    using state_t     = muse_mcl_2d::StateSpaceDescription2D::state_t;

    LikelihoodFieldModelVector();

    virtual void apply(const data_t::ConstPtr          &data,
                       const state_space_t::ConstPtr   &map,
                       sample_set_t::weight_iterator_t  set) override;

protected:
    std::size_t max_beams_;
    double      z_hit_;
    double      z_rand_;
    double      sigma_hit_;
    double      denominator_exponent_hit_;
    double      denominator_hit_;

    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // LIKELIHOOD_FIELD_MODEL_VECTOR_H
