#ifndef OMNI_DRIVE_H
#define OMNI_DRIVE_H

#include <cslibs_math/random/random.hpp>

#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>

namespace muse_mcl_2d_odometry {
class OmniDrive : public muse_mcl_2d::PredictionModel2D
{
public:
    OmniDrive() = default;

    virtual Result::Ptr apply(const cslibs_plugins_data::Data::ConstPtr          &data,
                              const cslibs_time::Time                    &until,
                              sample_set_t::state_iterator_t  states) override;

protected:
    unsigned int seed_;
    double alpha_1_;
    double alpha_2_;
    double alpha_3_;
    double alpha_4_;
    double alpha_5_;

    cslibs_math::random::Normal<1>::Ptr rng_delta_trans_hat_;
    cslibs_math::random::Normal<1>::Ptr rng_delta_rot_hat_;
    cslibs_math::random::Normal<1>::Ptr rng_delta_strafe_hat_;

    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif /* OMNI_DRIVE_H */
