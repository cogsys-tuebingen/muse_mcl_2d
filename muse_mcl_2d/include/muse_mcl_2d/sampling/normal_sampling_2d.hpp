#ifndef MUSE_MCL_2D_NORMAL_2D_HPP
#define MUSE_MCL_2D_NORMAL_2D_HPP

#include <ros/time.h>

#include <cslibs_math_ros/tf/tf_provider.hpp>
#include <cslibs_plugins/plugin.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_smc/smc/traits/normal_sampling.hpp>

namespace muse_mcl_2d {
class NormalSampling2D
    : public muse_smc::traits::NormalSampling<Sample2D>::type,
      public cslibs_plugins::Plugin {
 public:
  using Ptr = std::shared_ptr<NormalSampling2D>;
  using map_providers_t = std::map<std::string, MapProvider2D::Ptr>;
  using tf_provider_t = cslibs_math_ros::tf::TFProvider;
  using sample_set_t = muse_smc::traits::SampleSet<Sample2D>::type;

  static std::string Type() { return "muse_mcl_2d::NormalSampling2D"; }

  inline NormalSampling2D() = default;
  virtual ~NormalSampling2D() = default;

  inline void setup(const map_providers_t &map_providers,
                    const tf_provider_t::Ptr &tf, ros::NodeHandle &nh) {
    auto param_name = [this](const std::string &name) {
      return name_ + "/" + name;
    };
    sample_size_ =
        static_cast<std::size_t>(nh.param(param_name("sample_size"), 500));
    sampling_timeout_ =
        ros::Duration(nh.param(param_name("sampling_timeout"), 10.0));
    tf_timeout_ = ros::Duration(nh.param(param_name("tf_timeout"), 0.1));
    tf_ = tf;

    doSetup(map_providers, nh);
  }

 protected:
  std::size_t sample_size_{500};
  ros::Duration sampling_timeout_{10.0};
  ros::Duration tf_timeout_{0.1};
  tf_provider_t::Ptr tf_{nullptr};

  virtual void doSetup(const map_providers_t &map_providers,
                       ros::NodeHandle &nh) = 0;
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_NORMAL_2D_HPP
