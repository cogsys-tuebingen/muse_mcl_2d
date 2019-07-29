#include <muse_mcl_2d/impl/resampling/systematic_2d.hpp>

#include <muse_smc/resampling/impl/systematic.hpp>

namespace muse_mcl_2d {

void Systematic::doApply(sample_set_t& sample_set)
{
    muse_smc::impl::Systematic<muse_smc::SMC<Sample2D>>::apply(sample_set);
}

void Systematic::doApplyRecovery(sample_set_t& sample_set)
{
    muse_smc::impl::Systematic<muse_smc::SMC<Sample2D>>::applyRecovery(uniform_pose_sampler_,
            recovery_random_pose_probability_,
            sample_set);
}
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Systematic, muse_mcl_2d::Resampling2D)
