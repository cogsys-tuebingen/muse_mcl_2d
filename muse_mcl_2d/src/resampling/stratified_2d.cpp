#include <muse_mcl_2d/impl/resampling/stratified_2d.hpp>

#include <muse_smc/resampling/impl/stratified.hpp>

namespace muse_mcl_2d {


void Stratified::doApply(sample_set_t& sample_set)
{
    muse_smc::impl::Stratified<sample_set_t, uniform_sampling_t>::apply(sample_set);
}

void Stratified::doApplyRecovery(sample_set_t& sample_set)
{
    muse_smc::impl::Stratified<sample_set_t, uniform_sampling_t>::applyRecovery(uniform_pose_sampler_,
            recovery_random_pose_probability_,
            sample_set);
}
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::Stratified, muse_mcl_2d::Resampling2D)
