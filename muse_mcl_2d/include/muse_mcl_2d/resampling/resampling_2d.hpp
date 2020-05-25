#ifndef MUSE_MCL_2D_RESAMPLING_2D_HPP
#define MUSE_MCL_2D_RESAMPLING_2D_HPP

#include <ros/ros.h>
#include <class_loader/register_macro.hpp>

#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_smc/smc/traits/resampling.hpp>

namespace muse_mcl_2d {
class Resampling2D : public muse_smc::traits::Resampling<Sample2D>::type,
                     public cslibs_plugins::Plugin {
 public:
  using Ptr = std::shared_ptr<Resampling2D>;
  using base_t = muse_smc::traits::Resampling<Sample2D>::type;
  using uniform_sampling_t = muse_smc::traits::UniformSampling<Sample2D>::type;
  using normal_sampling_t = muse_smc::traits::NormalSampling<Sample2D>::type;
  using sample_set_t = muse_smc::traits::SampleSet<Sample2D>::type;

  static std::string Type() { return "muse_mcl_2d::Resampling2D"; }

  inline void setup(
      const typename uniform_sampling_t::Ptr &uniform_pose_sampler,
      const typename normal_sampling_t::Ptr &normal_pose_sampler,
      ros::NodeHandle &nh) {
    auto param_name = [this](const std::string &name) {
      return name_ + "/" + name;
    };
    base_t::setup(uniform_pose_sampler, normal_pose_sampler,
                  nh.param(param_name("recovery_alpha_fast"), 0.0),
                  nh.param(param_name("recovery_alpha_slow"), 0.0),
                  nh.param(param_name("variance_threshold"), 0.0));
    doSetup(nh);
  }

 protected:
  virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_RESAMPLING_2D_HPP
