#pragma once

#include <muse_mcl_2d/resampling/resampling_2d.hpp>

namespace muse_mcl_2d {
class Residual : public Resampling2D {
 public:
    Residual() = default;
 protected:
  virtual void doSetup(ros::NodeHandle &nh) override {}
  virtual void doApply(sample_set_t &sample_set) override;
  virtual void doApplyRecovery(sample_set_t &sample_set) override;
};
}  // namespace muse_mcl_2d
