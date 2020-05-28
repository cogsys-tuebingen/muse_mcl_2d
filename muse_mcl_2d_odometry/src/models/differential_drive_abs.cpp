#include <muse_mcl_2d_odometry/models/differential_drive_abs.h>

#include <class_loader/register_macro.hpp>
#include <cslibs_math/common/angle.hpp>
#include <cslibs_plugins_data/types/odometry_2d.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_odometry::DifferentialDriveAbs,
                            muse_mcl_2d::PredictionModel2D)

namespace muse_mcl_2d_odometry {
std::shared_ptr<DifferentialDriveAbs::Result> DifferentialDriveAbs::apply(
    const std::shared_ptr<data_t const> &data, const time_t &until,
    sample_set_t::state_iterator_t states) {
  /// t^O_0 ------------------ t^O_1
  /// ---------- t^s
  /// ----------------- t^u
  ///

  Result2D::odometry_t::ConstPtr original =
      std::dynamic_pointer_cast<Result2D::odometry_t const>(data);

  Result2D::odometry_t::ConstPtr apply = original;
  Result2D::odometry_t::ConstPtr leave;

  const cslibs_time::Time &s = states.getStamp();
  const cslibs_time::TimeFrame &tf = data->timeFrame();

  original = s > tf.start ? original->cutFront(s)
                          : original;      /// try to cut of the front
  original = original ? apply : original;  /// if fails reset to original

  if (until < tf.end &&
      !original->split(until, apply, leave)) {  /// if message has to be split
    apply = original;
  }

  const Result2D::odometry_t &odometry = *apply;

  const double delta_trans = odometry.getDeltaLinear();
  double delta_rot1 = 0.0;
  if (delta_trans >= translation_threshold_) {
    delta_rot1 = cslibs_math::common::angle::difference(
        odometry.getDeltaAngularAbs(), odometry.getStartPose().yaw());
  }
  const double delta_rot2 = cslibs_math::common::angle::difference(
      odometry.getDeltaAngular(), delta_rot1);

  if (delta_trans < eps_zero_linear_ &&
      std::abs(delta_rot2) < eps_zero_angular_) {
    return std::shared_ptr<Result>(new Result2D(0.0, 0.0, apply, leave));
  }

  const double delta_rot_noise1 = std::min(
      std::abs(cslibs_math::common::angle::difference(delta_rot1, 0.0)),
      std::abs(cslibs_math::common::angle::difference(delta_rot1, M_PI)));
  const double delta_rot_noise2 = std::min(
      std::abs(cslibs_math::common::angle::difference(delta_rot2, 0.0)),
      std::abs(cslibs_math::common::angle::difference(delta_rot2, M_PI)));
  const double sigma_rot_hat1 = std::sqrt(
      alpha_1_ * std::abs(delta_rot_noise1) + alpha_2_ * std::abs(delta_trans));
  const double sigma_trans_hat = std::sqrt(
      alpha_3_ * std::abs(delta_trans) + alpha_4_ * std::abs(delta_rot_noise1) +
      alpha_4_ * std::abs(delta_rot_noise2));
  const double sigma_rot_hat2 = std::sqrt(
      alpha_1_ * std::abs(delta_rot_noise2) + alpha_2_ * std::abs(delta_trans));

  if (!rng_delta_rot_hat1_) {
    rng_delta_rot_hat1_.reset(
        new cslibs_math::random::Normal<double, 1>(0.0, sigma_rot_hat1, seed_));
  } else {
    rng_delta_rot_hat1_->set(0.0, sigma_rot_hat1);
  }
  if (!rng_delta_trans_hat_) {
    rng_delta_trans_hat_.reset(new cslibs_math::random::Normal<double, 1>(
        0.0, sigma_trans_hat, seed_ + 1));
  } else {
    rng_delta_trans_hat_->set(0.0, sigma_trans_hat);
  }
  if (!rng_delta_rot_hat2_) {
    rng_delta_rot_hat2_.reset(new cslibs_math::random::Normal<double, 1>(
        0.0, sigma_rot_hat2, seed_ + 2));
  } else {
    rng_delta_rot_hat2_->set(0.0, sigma_rot_hat2);
  }

  for (muse_mcl_2d::Sample2D::state_t &sample : states) {
    const double delta_rot_hat1 = cslibs_math::common::angle::difference(
        delta_rot1, rng_delta_rot_hat1_->get());
    const double delta_trans_hat = delta_trans - rng_delta_trans_hat_->get();
    const double delta_rot_hat2 = cslibs_math::common::angle::difference(
        delta_rot2, rng_delta_rot_hat2_->get());
    const double tx =
        sample.tx() + delta_trans_hat * std::cos(sample.yaw() + delta_rot_hat1);
    const double ty =
        sample.ty() + delta_trans_hat * std::sin(sample.yaw() + delta_rot_hat1);
    const double yaw = cslibs_math::common::angle::normalize(
        sample.yaw() + delta_rot_hat1 + delta_rot_hat2);
    sample.setFrom(tx, ty, yaw);
  }
  return std::shared_ptr<Result>(
      new Result2D(delta_trans, std::abs(delta_rot2), apply, leave));
}

void DifferentialDriveAbs::doSetup(ros::NodeHandle &nh_private) {
  auto param_name = [this](const std::string &name) {
    return name_ + "/" + name;
  };

  seed_ = nh_private.param<int>(param_name("seed"), 0);
  alpha_1_ = nh_private.param<double>(param_name("alpha1"), 0.1);
  alpha_2_ = nh_private.param<double>(param_name("alpha2"), 0.1);
  alpha_3_ = nh_private.param<double>(param_name("alpha3"), 0.1);
  alpha_4_ = nh_private.param<double>(param_name("alpha4"), 0.1);
  translation_threshold_ =
      nh_private.param<double>(param_name("translation_threshold"), 0.001);
  eps_zero_angular_ = nh_private.param<double>(param_name("eps_linear"), 1e-6);
  eps_zero_linear_ = nh_private.param<double>(param_name("eps_angular"), 1e-6);
}
}  // namespace muse_mcl_2d_odometry
