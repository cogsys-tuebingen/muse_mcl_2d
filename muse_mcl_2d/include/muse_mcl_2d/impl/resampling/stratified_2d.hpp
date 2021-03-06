#pragma once

#include <muse_mcl_2d/resampling/resampling_2d.hpp>

namespace muse_mcl_2d {
class Stratified : public Resampling2D {
 public:
    Stratified() = default;
 protected:
  void doSetup(ros::NodeHandle &nh) override {}
  void doApply(sample_set_t &sample_set) override;
  void doApplyRecovery(sample_set_t &sample_set) override;
};
}  // namespace muse_mcl_2d
