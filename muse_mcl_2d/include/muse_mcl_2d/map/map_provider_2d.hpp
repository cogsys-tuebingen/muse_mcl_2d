#ifndef MUSE_MCL_2D_MAP_PROVIDER_2D_HPP
#define MUSE_MCL_2D_MAP_PROVIDER_2D_HPP

#include <ros/ros.h>
#include <cslibs_plugins/plugin.hpp>

#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_smc/smc/traits/state_space_provider.hpp>

namespace muse_mcl_2d {
class MapProvider2D
    : public muse_smc::traits::StateSpaceProvider<Sample2D>::type,
      public cslibs_plugins::Plugin {
 public:
  using Ptr = std::shared_ptr<MapProvider2D>;
  using ConstPtr = std::shared_ptr<MapProvider2D const>;

  static std::string Type() { return "muse_mcl_2d::MapProvider2D"; }

  std::string const& getName() const override {
    return cslibs_plugins::Plugin::getName();
  }

  virtual void setup(ros::NodeHandle& nh) = 0;
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_MAP_PROVIDER_2D_HPP
