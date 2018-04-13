#ifndef UPDATE_MODEL2D_HPP
#define UPDATE_MODEL2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/state_space/state_space_description_2d.hpp>

#include <muse_smc/update/update_model.hpp>

#include <cslibs_math_ros/tf/tf_listener_2d.hpp>

#include <cslibs_plugins_data/data.hpp>

namespace muse_mcl_2d {
class UpdateModel2D : public muse_smc::UpdateModel<StateSpaceDescription2D, cslibs_plugins_data::Data>
{
public:
    using Ptr    = std::shared_ptr<UpdateModel2D>;
    using data_t = cslibs_plugins_data::Data;

    inline const static std::string Type()
    {
        return "muse_mcl_2d::UpdateModel2D";
    }

    inline UpdateModel2D()
    {
    }

    virtual ~UpdateModel2D()
    {
    }

    inline void setup(const cslibs_math_ros::tf::TFListener2d::Ptr &tf,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        tf_ = tf;

        world_frame_      = nh.param<std::string>("world_frame", "world");
        robot_base_frame_ = nh.param<std::string>("base_frame",  "base_link");
        tf_timeout_       = ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));
        doSetup(nh);
    }

protected:
    cslibs_math_ros::tf::TFListener2d::Ptr tf_;
    ros::Duration tf_timeout_;
    std::string   world_frame_;
    std::string   robot_base_frame_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}

#endif // UPDATE_MODEL2D_HPP
