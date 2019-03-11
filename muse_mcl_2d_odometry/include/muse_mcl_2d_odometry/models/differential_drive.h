#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <cslibs_math/random/random.hpp>

#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>

namespace muse_mcl_2d_odometry {
class DifferentialDrive : public muse_mcl_2d::PredictionModel2D
{
public:
    DifferentialDrive() = default;

    virtual Result::Ptr apply(const cslibs_plugins_data::Data::ConstPtr &data,
                              const cslibs_time::Time                   &until,
                              sample_set_t::state_iterator_t             states) override;

protected:
    unsigned int seed_;
    double       alpha_1_;
    double       alpha_2_;
    double       alpha_3_;
    double       alpha_4_;
    double       alpha_5_;
    double       translation_threshold_;

    cslibs_math::random::Normal<double,1>::Ptr rng_delta_rot_hat1_;
    cslibs_math::random::Normal<double,1>::Ptr rng_delta_trans_hat_;
    cslibs_math::random::Normal<double,1>::Ptr rng_delta_rot_hat2_;

    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif /* DIFFERENTIAL_DRIVE_H */
