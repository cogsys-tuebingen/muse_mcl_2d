#ifndef BEAM_MODEL_MLE_H
#define BEAM_MODEL_MLE_H

#include <atomic>
#include <muse_mcl_2d/update/update_model_2d.hpp>

#include "beam_model_parameter_estimator.h"

namespace muse_mcl_2d_gridmaps {
class BeamModelMLE : public muse_mcl_2d::UpdateModel2D {
 public:
  BeamModelMLE();

  using base_t = muse_mcl_2d::UpdateModel2D;
  using base_t::sample_set_t;
  using base_t::state_space_t;
  using base_t::data_t;

  virtual void apply(const std::shared_ptr<data_t const> &data,
                     const std::shared_ptr<state_space_t const> &map,
                     sample_set_t::weight_iterator_t set) override;

 protected:
  std::size_t max_beams_;

  BeamModelParameterEstimator::Ptr parameter_estimator_mle_;
  BeamModelParameterEstimator::Parameters parameters_;
  bool use_estimated_parameters_;
  bool use_weights_for_estimation_;

  virtual void doSetup(ros::NodeHandle &nh) override;
};
}  // namespace muse_mcl_2d_gridmaps

#endif  // BEAM_MODEL_MLE_H
