#ifndef MAP_PROVIDER_2D_HPP
#define MAP_PROVIDER_2D_HPP

#include <ros/ros.h>

#include <muse_smc/state_space/state_space_provider.hpp>
#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/state_space/state_space_description_2d.hpp>

#include <cslibs_plugins/plugin.hpp>

namespace muse_mcl_2d {
class MapProvider2D : public muse_smc::StateSpaceProvider<StateSpaceDescription2D>,
                      public cslibs_plugins::Plugin
{
public:
    using Ptr      = std::shared_ptr<MapProvider2D>;
    using ConstPtr = std::shared_ptr<MapProvider2D const>;

    inline const static std::string Type()
    {
        return "muse_mcl_2d::MapProvider2D";
    }

    virtual inline const std::string getName() const override
    {
        return cslibs_plugins::Plugin::getName();
    }

    virtual void setup(ros::NodeHandle &nh) = 0;
};
}

#endif // MAP_PROVIDER_2D_HPP
