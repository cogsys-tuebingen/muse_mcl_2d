#include <muse_mcl_2d_odometry/models/omni_drive.h>

#include <class_loader/register_macro.hpp>
#include <cslibs_math/common/angle.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_odometry::OmniDrive,
                            muse_mcl_2d::PredictionModel2D)

namespace muse_mcl_2d_odometry {
std::shared_ptr<OmniDrive::Result> OmniDrive::apply(
    const std::shared_ptr<data_t const> &data, const time_t &until,
    sample_set_t::state_iterator_t states) {
  Result2D::odometry_t::ConstPtr apply;
  Result2D::odometry_t::ConstPtr leave;
  if (until < data->timeFrame().end) {
    Result2D::odometry_t::ConstPtr original =
        std::dynamic_pointer_cast<Result2D::odometry_t const>(data);
    if (!original->split(until, apply, leave)) {
      apply = original;
    }
  } else {
    apply = std::dynamic_pointer_cast<Result2D::odometry_t const>(data);
  }

  const Result2D::odometry_t &odometry = *apply;

  const double delta_trans = odometry.getDeltaLinear();
  const double delta_rot = odometry.getDeltaAngular();

  if (delta_trans < eps_zero_linear_ && delta_rot < eps_zero_angular_)
    return std::shared_ptr<Result>(new Result2D(0.0, 0.0, apply, leave));

  auto sq = [](const double x) { return x * x; };

  const double delta_angle = odometry.getDeltaAngularAbs();
  const double delta_trans_hat_stddev =
      std::sqrt(alpha_3_ * sq(delta_trans) + alpha_1_ * sq(delta_rot));
  const double delta_rot_hat_stddev =
      std::sqrt(alpha_4_ * sq(delta_rot) + alpha_2_ * sq(delta_trans));
  const double delta_strafe_hat_stddev =
      std::sqrt(alpha_1_ * sq(delta_rot) + alpha_5_ * sq(delta_trans));
  if (!rng_delta_trans_hat_) {
    rng_delta_trans_hat_.reset(new cslibs_math::random::Normal<double, 1>(
        0.0, delta_trans_hat_stddev, seed_));
  } else {
    rng_delta_trans_hat_->set(0.0, delta_trans_hat_stddev);
  }
  if (!rng_delta_rot_hat_) {
    rng_delta_rot_hat_.reset(new cslibs_math::random::Normal<double, 1>(
        0.0, delta_rot_hat_stddev, seed_));
  } else {
    rng_delta_rot_hat_->set(0.0, delta_rot_hat_stddev);
  }
  if (!rng_delta_strafe_hat_) {
    rng_delta_strafe_hat_.reset(new cslibs_math::random::Normal<double, 1>(
        0.0, delta_strafe_hat_stddev, seed_));
  } else {
    rng_delta_strafe_hat_->set(0.0, delta_strafe_hat_stddev);
  }

  for (muse_mcl_2d::Sample2D::state_t &sample : states) {
    double tx = sample.tx();
    double ty = sample.ty();
    double yaw = sample.yaw();

    const double delta_bearing =
        cslibs_math::common::angle::difference(delta_angle,
                                               odometry.getStartPose().yaw()) +
        yaw;

    const double cos_delta_bearing = std::cos(delta_bearing);
    const double sin_delta_bearing = std::sin(delta_bearing);
    const double delta_trans_hat = delta_trans + rng_delta_trans_hat_->get();
    const double delta_rot_hat = delta_rot + rng_delta_rot_hat_->get();
    const double delta_strafe_hat = 0.0 + rng_delta_strafe_hat_->get();
    tx += (delta_trans_hat * cos_delta_bearing +
           delta_strafe_hat * sin_delta_bearing);
    ty += (delta_trans_hat * sin_delta_bearing -
           delta_strafe_hat * cos_delta_bearing);
    yaw += delta_rot_hat;
    sample.setFrom(tx, ty, yaw);
  }

  return std::shared_ptr<Result>(
      new Result2D(delta_trans, delta_rot, apply, leave));
}

void OmniDrive::doSetup(ros::NodeHandle &nh_private) {
  auto param_name = [this](const std::string &name) {
    return name_ + "/" + name;
  };

  seed_ = nh_private.param<int>(param_name("seed"), 0);
  alpha_1_ = nh_private.param<double>(param_name("alpha1"), alpha_1_);
  alpha_2_ = nh_private.param<double>(param_name("alpha1"), alpha_2_);
  alpha_3_ = nh_private.param<double>(param_name("alpha1"), alpha_3_);
  alpha_4_ = nh_private.param<double>(param_name("alpha1"), alpha_4_);
  alpha_5_ = nh_private.param<double>(param_name("alpha5"), 0.1);
}
}  // namespace muse_mcl_2d_odometry
