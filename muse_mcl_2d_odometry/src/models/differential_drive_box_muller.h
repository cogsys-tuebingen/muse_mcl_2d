#ifndef DIFFERENTIAL_DRIVE_BOX_MULLER_H
#define DIFFERENTIAL_DRIVE_BOX_MULLER_H

#include <cslibs_math/random/random.hpp>

#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>

namespace muse_mcl_2d_odometry {
class DifferentialDriveBoxMuller : public muse_mcl_2d::PredictionModel2D
{
public:
    DifferentialDriveBoxMuller() = default;

    virtual Result::Ptr apply(const cslibs_plugins_data::Data::ConstPtr  &data,
                              const cslibs_time::Time                    &until,
                              sample_set_t::state_iterator_t  states) override;

protected:
    unsigned int seed_;
    double       alpha_1_;
    double       alpha_2_;
    double       alpha_3_;
    double       alpha_4_;
    double       alpha_5_;
    double       translation_threshold_;

    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // DIFFERENTIAL_DRIVE_BOX_MULLER_H
