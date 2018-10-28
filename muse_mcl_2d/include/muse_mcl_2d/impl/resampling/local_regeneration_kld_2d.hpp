#pragma once

#include <cslibs_math/random/random.hpp>

#include <muse_mcl_2d/resampling/resampling_2d.hpp>
#include <muse_mcl_2d/density/sample_density_2d.hpp>

namespace muse_mcl_2d {
class LocalRegenerationKLD2D : public Resampling2D
{

protected:
    double                       kld_error_;
    double                       kld_z_;
    cslibs_math_2d::Covariance3d covariance_;
    double                       cov_x_;
    double                       cov_y_;
    double                       cov_yaw_;
    double                       variance_threshold_;
    bool                         reset_one_;

    virtual void doSetup(ros::NodeHandle &nh) override;
    void doApply(sample_set_t &sample_set);
    void doApplyRecovery(sample_set_t &sample_set);

};
}
