#include <muse_mcl_2d/impl/resampling/residual_2d.hpp>

#include <muse_smc/resampling/impl/residual.hpp>

namespace muse_mcl_2d {

void Residual::doApply(sample_set_t& sample_set)
{
    muse_smc::impl::Residual<Sample2D>::apply(sample_set);
}

void Residual::doApplyRecovery(sample_set_t& sample_set)
{
    muse_smc::impl::Residual<Sample2D>::applyRecovery(uniform_pose_sampler_,
            recovery_random_pose_probability_,
            sample_set);
}
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Residual, muse_mcl_2d::Resampling2D)
