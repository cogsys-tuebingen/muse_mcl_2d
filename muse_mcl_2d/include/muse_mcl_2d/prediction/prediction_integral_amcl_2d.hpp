#ifndef MUSE_MCL_2D_PREDICTION_INTEGRAL_AMCL_2D_HPP
#define MUSE_MCL_2D_PREDICTION_INTEGRAL_AMCL_2D_HPP

#include <cslibs_plugins_data/data.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>
#include <muse_smc/prediction/prediction_integral.hpp>

namespace muse_mcl_2d {
class PredictionIntegralAMCL2D : public muse_smc::traits::PredictionIntegrals<
                                     Sample2D>::prediction_integral_t {
 public:
  using pose_t = muse_smc::traits::Transform<Sample2D>::type;

  inline PredictionIntegralAMCL2D() = default;

  inline explicit PredictionIntegralAMCL2D(const double linear_threshold,
                                           const double angular_threshold,
                                           const bool exceed_both = false)
      : linear_threshold_{linear_threshold} angular_threshold_{
            angular_threshold} exceed_both_{exceed_both} {}

  virtual ~PredictionIntegralAMCL2D() = default;

  void add(const Result::ConstPtr &step) override {
    if (step->isType<PredictionModel2D::Result2D>()) {
      const PredictionModel2D::Result2D &step_2d =
          step->as<PredictionModel2D::Result2D>();
      const cslibs_plugins_data::types::Odometry2d &appl =
          step_2d.applied->as<const cslibs_plugins_data::types::Odometry2d>();
      if (start_pose_.tx() == 0.0 && start_pose_.ty() == 0.0 &&
          start_pose_.yaw() == 0.0)
        start_pose_ = appl.getStartPose();
      end_pose_ = appl.getEndPose();

      linear_distance_x_abs_ = std::fabs(end_pose_.tx() - start_pose_.tx());
      linear_distance_y_abs_ = std::fabs(end_pose_.ty() - start_pose_.ty());
      angular_distance_abs_ = std::fabs(cslibs_math::common::angle::difference(
          end_pose_.yaw(), start_pose_.yaw()));
    } else {
      throw std::runtime_error(
          "PreditionIntegral is fed the wrong prediction step type!");
    }
  }

  void reset() override {
    start_pose_ = end_pose_;
    linear_distance_x_abs_ = 0.0;
    linear_distance_y_abs_ = 0.0;
    angular_distance_abs_ = 0.0;
  }

  bool thresholdExceeded() const override {
    if (exceed_both_)
      return (linear_distance_x_abs_ >= linear_threshold_ &&
              linear_distance_y_abs_ >= linear_threshold_ &&
              angular_distance_abs_ >= angular_threshold_);

    return (linear_distance_x_abs_ >= linear_threshold_ ||
            linear_distance_y_abs_ >= linear_threshold_ ||
            angular_distance_abs_ >= angular_threshold_);
  }

  bool isZero() const override {
    return linear_distance_x_abs_ == 0.0 && linear_distance_y_abs_ == 0.0 &&
           angular_distance_abs_ == 0.0;
  }

  void info() const override {
    std::cerr << linear_distance_x_abs_ << " " << linear_distance_y_abs_ << " "
              << angular_distance_abs_ << "\n";
  }

 private:
  double linear_distance_x_abs_{0.0};
  double linear_distance_y_abs_{0.0};
  double angular_distance_abs_{0.0};

  double linear_threshold_{0.0};
  double angular_threshold_{0.0};

  bool exceed_both_{false};

  pose_t start_pose_;
  pose_t end_pose_;
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_PREDICTION_INTEGRAL_AMCL_2D_HPP
