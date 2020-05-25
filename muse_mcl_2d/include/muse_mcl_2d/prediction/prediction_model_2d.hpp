#ifndef MUSE_MCL_2D_PREDICTION_MODEL_2D_HPP
#define MUSE_MCL_2D_PREDICTION_MODEL_2D_HPP

#include <ros/node_handle.h>

#include <cslibs_math_ros/tf/tf_provider.hpp>
#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data.hpp>
#include <cslibs_plugins_data/types/odometry_2d.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_smc/smc/traits/prediction_model.hpp>

namespace muse_mcl_2d {
class PredictionModel2D
    : public muse_smc::traits::PredictionModel<Sample2D>::type,
      public cslibs_plugins::Plugin {
 public:
  using Ptr = std::shared_ptr<PredictionModel2D>;
  using ConstPtr = std::shared_ptr<PredictionModel2D const>;

  struct Result2D : public Result {
    using odometry_t = cslibs_plugins_data::types::Odometry2d;
    inline explicit Result2D(const double linear_distance_abs,
                             const double angular_distance_abs,
                             const odometry_t::ConstPtr &applied,
                             const odometry_t::ConstPtr &left_to_apply)
        : Result(applied, left_to_apply),
          linear_distance_abs{linear_distance_abs},
          angular_distance_abs{angular_distance_abs} {}

    const double linear_distance_abs;
    const double angular_distance_abs;
  };

  PredictionModel2D() = default;
  virtual ~PredictionModel2D() = default;

  static std::string Type() { return "muse_mcl_2d::PredictionModel2D"; }

  inline void setup(const cslibs_math_ros::tf::TFProvider::Ptr &tf,
                    ros::NodeHandle &nh) {
    auto param_name = [this](const std::string &name) {
      return name_ + "/" + name;
    };
    tf_ = tf;
    eps_zero_linear_ = nh.param(param_name("eps_zero_linear"), 1e-4);
    eps_zero_angular_ = nh.param(param_name("eps_zero_angular"), 1e-4);
    doSetup(nh);
  }

 protected:
  cslibs_math_ros::tf::TFProvider::Ptr tf_;
  double eps_zero_linear_{1e-4};
  double eps_zero_angular_{1e-4};

  virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}  // namespace muse_mcl_2d

#endif  // PREDICTION_MODEL_2D_HPP
