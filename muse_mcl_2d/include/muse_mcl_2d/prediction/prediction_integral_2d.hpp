#ifndef PREDICTION_INTEGRAL_2D_HPP
#define PREDICTION_INTEGRAL_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>

#include <muse_smc/prediction/prediction_integral.hpp>
#include <cslibs_plugins_data/data.hpp>

namespace muse_mcl_2d {
class PredictionIntegral2D : public muse_smc::PredictionIntegral<StateSpaceDescription2D, cslibs_plugins_data::Data>
{
public:
    inline PredictionIntegral2D() :
        linear_distance_abs_(0.0),
        angular_distance_abs_(0.0),
        resampling_linear_threshold_(0.0),
        resampling_angular_threshold_(0.0),
        update_linear_threshold_(0.0),
        update_angular_threshold_(0.0)
    {
    }

    inline PredictionIntegral2D(const double resampling_linear_threshold,
                                const double resampling_angular_threshold,
                                const double update_linear_threshold = 0.0,
                                const double update_angular_threshold = 0.0) :
        linear_distance_abs_(0.0),
        angular_distance_abs_(0.0),
        resampling_linear_threshold_(resampling_linear_threshold),
        resampling_angular_threshold_(resampling_angular_threshold),
        update_linear_threshold_(update_linear_threshold),
        update_angular_threshold_(update_angular_threshold)
    {
    }

    virtual ~PredictionIntegral2D()
    {
    }

    virtual void add(const typename prediction_model_t::Result::ConstPtr &step) override
    {
        if(step->isType<PredictionModel2D::Result2D>()) {
            const PredictionModel2D::Result2D &step_2d = step->as<PredictionModel2D::Result2D>();
            linear_distance_abs_ += step_2d.linear_distance_abs;
            angular_distance_abs_ += step_2d.angular_distance_abs;
        } else {
            throw std::runtime_error("PreditionIntegral is fed the wrong prediction step type!");
        }
    }

    virtual void reset() override
    {
        linear_distance_abs_ = 0.0;
        angular_distance_abs_ = 0.0;
    }

    virtual bool updateThresholdExceeded() const override
    {
        return linear_distance_abs_ > update_linear_threshold_ ||
                angular_distance_abs_ > update_angular_threshold_;
    }

    virtual bool resamplingThresholdExceeded() const override
    {
        return linear_distance_abs_ > resampling_linear_threshold_ ||
                angular_distance_abs_ > resampling_angular_threshold_;
    }

    virtual bool isZero() const override
    {
        return linear_distance_abs_ == 0.0 &&
                angular_distance_abs_ == 0.0;
    }

    virtual void info() const override
    {
        std::cerr << linear_distance_abs_ << " " << angular_distance_abs_ << "\n";
    }

private:
    double linear_distance_abs_;
    double angular_distance_abs_;

    double resampling_linear_threshold_;
    double resampling_angular_threshold_;

    double update_linear_threshold_;
    double update_angular_threshold_;
};
}

#endif // PREDICTION_INTEGRAL_2D_HPP
