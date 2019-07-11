#ifndef PREDICTION_MODEL_2D_HPP
#define PREDICTION_MODEL_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>

#include <muse_smc/prediction/prediction_model.hpp>

#include <cslibs_math_ros/tf/tf_provider.hpp>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/types/odometry_2d.hpp>
#include <cslibs_plugins_data/data.hpp>
#include <ros/node_handle.h>

namespace muse_mcl_2d {
class PredictionModel2D : public muse_smc::PredictionModel<Sample2D>,
                          public cslibs_plugins::Plugin
{
public:
    using Ptr = std::shared_ptr<PredictionModel2D>;

    struct Result2D : public Result
    {
        using odometry_t = cslibs_plugins_data::types::Odometry2d;
        inline Result2D(const double linear_distance_abs,
                        const double angular_distance_abs,
                        const odometry_t::ConstPtr &applied,
                        const odometry_t::ConstPtr &left_to_apply) :
            Result(applied, left_to_apply),
            linear_distance_abs(linear_distance_abs),
            angular_distance_abs(angular_distance_abs)
        {
        }

        const double linear_distance_abs;
        const double angular_distance_abs;
    };

    PredictionModel2D()
    {
    }

    virtual ~PredictionModel2D()
    {
    }

    inline const static std::string Type()
    {
        return "muse_mcl_2d::PredictionModel2D";
    }

    inline void setup(const cslibs_math_ros::tf::TFProvider::Ptr &tf,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        tf_ = tf;
        eps_zero_linear_  = nh.param(param_name("eps_zero_linear"),  1e-4);
        eps_zero_angular_ = nh.param(param_name("eps_zero_angular"), 1e-4);
        doSetup(nh);
    }

protected:
    cslibs_math_ros::tf::TFProvider::Ptr tf_;
    double eps_zero_linear_;
    double eps_zero_angular_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}

#endif // PREDICTION_MODEL_2D_HPP
