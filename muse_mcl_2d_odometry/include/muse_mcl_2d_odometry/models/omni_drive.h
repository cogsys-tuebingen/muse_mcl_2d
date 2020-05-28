#ifndef OMNI_DRIVE_H
#define OMNI_DRIVE_H

#include <cslibs_math/random/random.hpp>
#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>

namespace muse_mcl_2d_odometry {
class OmniDrive : public muse_mcl_2d::PredictionModel2D {
 public:
  OmniDrive() = default;

  using base_t = muse_mcl_2d::PredictionModel2D;
  using base_t::data_t;
  using base_t::sample_set_t;
  using base_t::time_t;

  virtual std::shared_ptr<Result> apply(
      const std::shared_ptr<data_t const> &data, const time_t &until,
      sample_set_t::state_iterator_t states) override;

 protected:
  unsigned int seed_;
  double alpha_1_;
  double alpha_2_;
  double alpha_3_;
  double alpha_4_;
  double alpha_5_;

  cslibs_math::random::Normal<double, 1>::Ptr rng_delta_trans_hat_;
  cslibs_math::random::Normal<double, 1>::Ptr rng_delta_rot_hat_;
  cslibs_math::random::Normal<double, 1>::Ptr rng_delta_strafe_hat_;

  virtual void doSetup(ros::NodeHandle &nh) override;
};
}  // namespace muse_mcl_2d_odometry

#endif /* OMNI_DRIVE_H */
