#ifndef MUSE_MCL_2D_PREDICTION_INTEGRAL_2D_HPP
#define MUSE_MCL_2D_PREDICTION_INTEGRAL_2D_HPP

#include <cslibs_plugins_data/data.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>
#include <muse_smc/smc/traits/prediction_integrals.hpp>

namespace muse_mcl_2d {
class PredictionIntegral2D : public muse_smc::PredictionIntegrals<
                                 Sample2D>::type::prediction_integral_t {
 public:
  inline PredictionIntegral2D() = default;

  inline explicit PredictionIntegral2D(const double linear_threshold,
                                       const double angular_threshold,
                                       const bool exceed_both = false)
      : linear_threshold_{linear_threshold},
        angular_threshold_{angular_threshold},
        exceed_both_{exceed_both} {}

  void add(const Result::ConstPtr &step) override {
    if (step->isType<PredictionModel2D::Result2D>()) {
      const PredictionModel2D::Result2D &step_2d =
          step->as<PredictionModel2D::Result2D>();
      linear_distance_abs_ += step_2d.linear_distance_abs;
      angular_distance_abs_ += step_2d.angular_distance_abs;
    } else {
      throw std::runtime_error(
          "PreditionIntegral is fed the wrong prediction step type!");
    }
  }

  void reset() override {
    linear_distance_abs_ = 0.0;
    angular_distance_abs_ = 0.0;
  }

  bool thresholdExceeded() const override {
    if (exceed_both_)
      return (linear_distance_abs_ >= linear_threshold_ &&
              angular_distance_abs_ >= angular_threshold_);

    return (linear_distance_abs_ >= linear_threshold_ ||
            angular_distance_abs_ >= angular_threshold_);
  }

  bool isZero() const override {
    return linear_distance_abs_ == 0.0 && angular_distance_abs_ == 0.0;
  }

  void info() const override {
    std::cerr << linear_distance_abs_ << " " << angular_distance_abs_ << "\n";
  }

 private:
  double linear_distance_abs_{0.0};
  double angular_distance_abs_{0.0};

  double linear_threshold_{0.0};
  double angular_threshold_{0.0};

  bool exceed_both_{false};
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_PREDICTION_INTEGRAL_2D_HPP
