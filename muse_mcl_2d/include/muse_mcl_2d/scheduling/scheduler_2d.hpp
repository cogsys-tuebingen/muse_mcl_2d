#ifndef SCHEDULER_2D_HPP
#define SCHEDULER_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>

#include <muse_smc/scheduling/scheduler.hpp>
#include <muse_mcl_2d/update/update_model_2d.hpp>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data.hpp>

#include <ros/ros.h>
#include <class_loader/class_loader_register_macro.h>


namespace muse_mcl_2d {
class Scheduler2D : public muse_smc::Scheduler<Sample2D>,
                    public cslibs_plugins::Plugin
{
public:
    using Ptr                 = std::shared_ptr<Scheduler2D>;
    using update_model_map_t  = std::map<std::string, UpdateModel2D::Ptr>;

    inline const static std::string Type()
    {
        return "muse_mcl_2d::Scheduler2D";
    }

    virtual void setup(const update_model_map_t &update_models,
                       ros::NodeHandle &nh) = 0;

};
}


#endif // SCHEDULER_2D_HPP
