#ifndef MUSE_MCL_2D_UPDATE_MODEL_2D_HPP
#define MUSE_MCL_2D_UPDATE_MODEL_2D_HPP

#include <ros/node_handle.h>

#include <cslibs_math_ros/tf/tf_provider.hpp>
#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_smc/smc/traits/update_model.hpp>

namespace muse_mcl_2d {
class UpdateModel2D : public muse_smc::traits::UpdateModel<Sample2D>::type,
                      public cslibs_plugins::Plugin {
 public:
  using Ptr = std::shared_ptr<UpdateModel2D>;
  using sample_t = Sample2D;
  using data_t = muse_smc::traits::Data<sample_t>::type;
  using transform_t = muse_smc::traits::Transform<sample_t>::type;
  using state_t = muse_smc::traits::State<sample_t>::type;
  using point_t = muse_smc::traits::StateSpaceBoundary<sample_t>::type;

  UpdateModel2D() = default;
  virtual ~UpdateModel2D() = default;

  static std::string Type() { return "muse_mcl_2d::UpdateModel2D"; }

  inline std::size_t getModelId() const override {
    return cslibs_plugins::Plugin::getId();
  }

  inline std::string const &getName() const override {
    return cslibs_plugins::Plugin::getName();
  }

  inline void setup(const cslibs_math_ros::tf::TFProvider::Ptr &tf,
                    ros::NodeHandle &nh) {
    auto param_name = [this](const std::string &name) {
      return name_ + "/" + name;
    };
    tf_ = tf;

    world_frame_ = nh.param<std::string>("world_frame", "world");
    robot_base_frame_ = nh.param<std::string>("base_frame", "base_link");
    tf_timeout_ =
        ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));
    doSetup(nh);
  }

 protected:
  cslibs_math_ros::tf::TFProvider::Ptr tf_{nullptr};
  ros::Duration tf_timeout_{0.1};
  std::string world_frame_;
  std::string robot_base_frame_;

  virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_UPDATE_MODEL_2D_HPP
