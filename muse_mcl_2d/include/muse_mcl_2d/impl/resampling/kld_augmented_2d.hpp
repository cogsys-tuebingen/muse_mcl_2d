#pragma once

#include <cslibs_math/random/random.hpp>

#include <muse_mcl_2d/resampling/resampling_2d.hpp>
#include <muse_mcl_2d/density/sample_density_2d.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace muse_mcl_2d {
class KLDAugmented2D : public Resampling2D
{
public:

protected:
    double  kld_error_;
    double  kld_z_;
    double  uniform_percent_;
    double  min_weight_ratio_;

    virtual void doSetup(ros::NodeHandle &nh) override;
    void doApply(sample_set_t &sample_set) override;
    void doApplyRecovery(sample_set_t &sample_set) override;
};
}
