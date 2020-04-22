#ifndef RESAMPLING_2D_HPP
#define RESAMPLING_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>


#include <ros/ros.h>
#include <class_loader/register_macro.hpp>
#include <muse_smc/smc/smc.hpp>

namespace muse_mcl_2d {
class Resampling2D : public muse_smc::SMC<Sample2D>::resampling_t,
                     public cslibs_plugins::Plugin
{
public:
    using Ptr    = std::shared_ptr<Resampling2D>;
    using base_t = muse_smc::SMC<Sample2D>::resampling_t;

    inline const static std::string Type()
    {
        return "muse_mcl_2d::Resampling2D";
    }

    inline void setup(const typename sample_uniform_t::Ptr &uniform_pose_sampler,
                      const typename sample_normal_t::Ptr  &normal_pose_sampler,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        base_t::setup(uniform_pose_sampler,
                      normal_pose_sampler,
                      nh.param(param_name("recovery_alpha_fast"), 0.0),
                      nh.param(param_name("recovery_alpha_slow"), 0.0),
                      nh.param(param_name("variance_thresold"), 0.0));
        doSetup(nh);
    }

protected:
    virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}

#endif // RESAMPLING_2D_HPP
