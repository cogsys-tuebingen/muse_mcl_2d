#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <cslibs_math/random/random.hpp>
#include <muse_mcl_2d/density/sample_density_2d.hpp>
#include <muse_mcl_2d/resampling/resampling_2d.hpp>

namespace muse_mcl_2d {
class KLD2D : public Resampling2D {
 public:
  KLD2D();

 protected:
  double kld_error_;
  double kld_z_;

  cslibs_math::random::Uniform<double, 1> rng_;

  virtual void doSetup(ros::NodeHandle &nh) override;
  void doApply(sample_set_t &sample_set) override;
  void doApplyRecovery(sample_set_t &sample_set) override;
};
}  // namespace muse_mcl_2d
