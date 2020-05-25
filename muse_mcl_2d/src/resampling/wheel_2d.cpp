#include <muse_mcl_2d/impl/resampling/wheel_2d.hpp>
#include <muse_smc/resampling/impl/wheel.hpp>


namespace muse_mcl_2d {

void WheelOfFortune::doApply(sample_set_t& sample_set) {
  muse_smc::impl::WheelOfFortune<sample_set_t, uniform_sampling_t>::apply(
      sample_set);
}

void WheelOfFortune::doApplyRecovery(sample_set_t& sample_set) {
  muse_smc::impl::WheelOfFortune<sample_set_t, uniform_sampling_t>::
      applyRecovery(uniform_pose_sampler_, recovery_random_pose_probability_,
                    sample_set);
}
}  // namespace muse_mcl_2d

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::WheelOfFortune,
                            muse_mcl_2d::Resampling2D)
