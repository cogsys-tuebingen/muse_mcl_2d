#ifndef LIKELIHOODFIELD_PROB_MODEL_AMCL_H
#define LIKELIHOODFIELD_PROB_MODEL_AMCL_H

#include <muse_mcl_2d/update/update_model_2d.hpp>

namespace muse_mcl_2d_gridmaps {
class LikelihoodFieldProbModelAMCL : public muse_mcl_2d::UpdateModel2D {
 public:
  LikelihoodFieldProbModelAMCL();

  using base_t = muse_mcl_2d::UpdateModel2D;
  using base_t::sample_set_t;
  using base_t::state_space_t;
  using base_t::data_t;

  virtual void apply(const std::shared_ptr<data_t const> &data,
                     const std::shared_ptr<state_space_t const> &map,
                     sample_set_t::weight_iterator_t set) override;

 protected:
  std::size_t max_beams_;
  double z_hit_;
  double z_rand_;
  double sigma_hit_;
  double denominator_hit_;
  bool beam_skip_;
  double beam_skip_distance_;
  double beam_skip_threshold_;
  double beam_skip_error_threshold_;

  std::vector<int> observation_histogram_;
  std::vector<char> observation_mask_;
  std::vector<double> observation_probability_buffer_;

  virtual void doSetup(ros::NodeHandle &nh) override;
};
}  // namespace muse_mcl_2d_gridmaps

#endif  // LIKELIHOODFIELD_PROB_MODEL_AMCL_H
