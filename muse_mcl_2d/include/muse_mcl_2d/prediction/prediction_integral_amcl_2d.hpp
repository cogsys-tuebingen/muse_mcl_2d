#ifndef PREDICTION_INTEGRAL_AMCL_2D_HPP
#define PREDICTION_INTEGRAL_AMCL_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>

#include <muse_smc/prediction/prediction_integral.hpp>
#include <cslibs_plugins_data/data.hpp>

namespace muse_mcl_2d {
class PredictionIntegralAMCL2D : public muse_smc::PredictionIntegral<Sample2D>
{
public:
    using pose_t = muse_smc::traits::Transform<Sample2D>::type;

    inline PredictionIntegralAMCL2D() :
        linear_distance_x_abs_(0.0),
        linear_distance_y_abs_(0.0),
        angular_distance_abs_(0.0),
        linear_threshold_(0.0),
        angular_threshold_(0.0),
        exceed_both_(false),
        start_pose_(pose_t()),
        end_pose_(pose_t())
    {
    }

    inline PredictionIntegralAMCL2D(const double linear_threshold,
                                    const double angular_threshold,
                                    const bool   exceed_both = false) :
        linear_distance_x_abs_(0.0),
        linear_distance_y_abs_(0.0),
        angular_distance_abs_(0.0),
        linear_threshold_(linear_threshold),
        angular_threshold_(angular_threshold),
        exceed_both_(exceed_both),
        start_pose_(pose_t()),
        end_pose_(pose_t())
    {
    }

    virtual ~PredictionIntegralAMCL2D()
    {
    }

    virtual void add(const typename prediction_model_t::Result::ConstPtr &step) override
    {
        if(step->isType<PredictionModel2D::Result2D>()) {
            const PredictionModel2D::Result2D &step_2d = step->as<PredictionModel2D::Result2D>();
            const cslibs_plugins_data::types::Odometry2d &appl = step_2d.applied->as<const cslibs_plugins_data::types::Odometry2d>();
            if (start_pose_.tx() == 0.0 && start_pose_.ty() == 0.0 && start_pose_.yaw() == 0.0)
                start_pose_ = appl.getStartPose();
            end_pose_ = appl.getEndPose();

            linear_distance_x_abs_ = std::fabs(end_pose_.tx() - start_pose_.tx());
            linear_distance_y_abs_ = std::fabs(end_pose_.ty() - start_pose_.ty());
            angular_distance_abs_  = std::fabs(cslibs_math::common::angle::difference(end_pose_.yaw(), start_pose_.yaw()));
        } else {
            throw std::runtime_error("PreditionIntegral is fed the wrong prediction step type!");
        }
    }

    virtual void reset() override
    {
        start_pose_ = end_pose_;
        linear_distance_x_abs_ = 0.0;
        linear_distance_y_abs_ = 0.0;
        angular_distance_abs_  = 0.0;
    }

    virtual bool thresholdExceeded() const override
    {
        if(exceed_both_)
          return (linear_distance_x_abs_ >= linear_threshold_ &&
                  linear_distance_y_abs_ >= linear_threshold_ &&
                  angular_distance_abs_  >= angular_threshold_);

        return  (linear_distance_x_abs_ >= linear_threshold_ ||
                 linear_distance_y_abs_ >= linear_threshold_ ||
                 angular_distance_abs_  >= angular_threshold_);
    }

    virtual bool isZero() const override
    {
        return linear_distance_x_abs_ == 0.0 &&
               linear_distance_y_abs_ == 0.0 &&
               angular_distance_abs_  == 0.0;
    }

    virtual void info() const override
    {
        std::cerr << linear_distance_x_abs_ << " " << linear_distance_y_abs_ << " " << angular_distance_abs_ << "\n";
    }

private:
    double linear_distance_x_abs_;
    double linear_distance_y_abs_;
    double angular_distance_abs_;

    double linear_threshold_;
    double angular_threshold_;

    bool   exceed_both_;

    pose_t start_pose_;
    pose_t end_pose_;
};
}

#endif // PREDICTION_INTEGRAL_AMCL_2D_HPP
