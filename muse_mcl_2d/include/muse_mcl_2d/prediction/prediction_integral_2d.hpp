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
        angular_distance_abs_(0.0)
    {
    }

    inline PredictionIntegral2D(const double linear_threshold,
                                const double angular_threshold) :
        linear_distance_abs_(0.0),
        angular_distance_abs_(0.0),
        linear_threshold_(linear_threshold),
        angular_threshold_(angular_threshold)
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

    virtual bool thresholdExceeded() const override
    {
        return linear_distance_abs_ > linear_threshold_
                || angular_distance_abs_ > angular_threshold_;
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

    double linear_threshold_;
    double angular_threshold_;
};
}

#endif // PREDICTION_INTEGRAL_2D_HPP
