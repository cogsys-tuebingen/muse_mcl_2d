#ifndef BEAM_MODEL_MLE_H
#define BEAM_MODEL_MLE_H

#include <atomic>

#include <muse_mcl_2d/update/update_model_2d.hpp>

#include "beam_model_parameter_estimator.h"

namespace muse_mcl_2d_gridmaps {
class BeamModelMLE : public muse_mcl_2d::UpdateModel2D
{
public:
    BeamModelMLE();

    virtual void apply(const data_t::ConstPtr          &data,
                       const state_space_t::ConstPtr   &map,
                       sample_set_t::weight_iterator_t  set) override;

protected:
    std::size_t                             max_beams_;

    BeamModelParameterEstimator::Ptr        parameter_estimator_mle_;
    BeamModelParameterEstimator::Parameters parameters_;
    bool                                    use_estimated_parameters_;
    bool                                    use_weights_for_estimation_;

    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // BEAM_MODEL_MLE_H
