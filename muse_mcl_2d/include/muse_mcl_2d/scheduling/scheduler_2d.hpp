#ifndef MUSE_MCL_2D_SCHEDULER_2D_HPP
#define MUSE_MCL_2D_SCHEDULER_2D_HPP

#include <ros/ros.h>

#include <class_loader/register_macro.hpp>
#include <cslibs_plugins/plugin.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_mcl_2d/update/update_model_2d.hpp>
#include <muse_smc/smc/traits/scheduler.hpp>

namespace muse_mcl_2d {
class Scheduler2D : public muse_smc::traits::Scheduler<Sample2D>::type,
                    public cslibs_plugins::Plugin {
 public:
  using Ptr = std::shared_ptr<Scheduler2D>;
  using update_model_map_t = std::map<std::string, UpdateModel2D::Ptr>;

  static std::string Type() { return "muse_mcl_2d::Scheduler2D"; }

  virtual void setup(const update_model_map_t &update_models,
                     ros::NodeHandle &nh) = 0;
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_SCHEDULER_2D_HPP
