#include <muse_mcl_2d/impl/resampling/wheel_2d.hpp>

#include <muse_smc/resampling/impl/wheel.hpp>

namespace muse_mcl_2d {

void WheelOfFortune::doApply(sample_set_t& sample_set)
{
    muse_smc::impl::WheelOfFortune<StateSpaceDescription2D>::apply(sample_set);
}

void WheelOfFortune::doApplyRecovery(sample_set_t& sample_set)
{
    muse_smc::impl::WheelOfFortune<StateSpaceDescription2D>::applyRecovery(uniform_pose_sampler_,
            recovery_random_pose_probability_,
            sample_set);
}
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::WheelOfFortune, muse_mcl_2d::Resampling2D)
