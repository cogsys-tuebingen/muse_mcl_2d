#include "likelihood_field_model_pc.h"


#include <cslibs_plugins_data/types/laserscan.hpp>
#include <muse_mcl_2d_gridmaps/maps/likelihood_field_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldModelPC, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_gridmaps {
LikelihoodFieldModelPC::LikelihoodFieldModelPC()
{
}

void LikelihoodFieldModelPC::apply(const data_t::ConstPtr          &data,
                                   const state_space_t::ConstPtr   &map,
                                   sample_set_t::weight_iterator_t set)
{
    if(!map->isType<LikelihoodFieldGridmap>()) {
        return;
    }

    const cslibs_gridmaps::static_maps::LikelihoodFieldGridmap   &gridmap    = *(map->as<LikelihoodFieldGridmap>().data());
    const cslibs_plugins_data::types::Laserscan                         &laser_data = data->as<cslibs_plugins_data::types::Laserscan>();
    const cslibs_plugins_data::types::Laserscan::rays_t                 &laser_rays = laser_data.getRays();

    /// laser to base transform
    cslibs_math_2d::Transform2d b_T_l;
    cslibs_math_2d::Transform2d m_T_w;
    if(!tf_->lookupTransform(robot_base_frame_,
                             laser_data.getFrame(),
                             ros::Time(laser_data.getTimeFrame().end.seconds()),
                             b_T_l,
                             tf_timeout_))
        return;
    if(!tf_->lookupTransform(world_frame_,
                             map->getFrame(),
                             ros::Time(laser_data.getTimeFrame().end.seconds()),
                             m_T_w,
                             tf_timeout_))
        return;

    const cslibs_plugins_data::types::Laserscan::rays_t rays = laser_data.getRays();
    const auto end = set.end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, (rays_size - 1) / (max_beams_ - 1));
    const double range_max      = laser_data.getLinearMax();
    const double p_rand         = z_rand_ * 1.0 / range_max;

    for(auto it = set.begin() ; it != end ; ++it) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            const cslibs_math_2d::Point2d ray_end_point = m_T_l * ray.end_point;
            const double pz = ray.valid() ? z_hit_ * gridmap.at(ray_end_point) + p_rand : 1.0;
            p *= pz;
        }
        *it *= p;
    }
}

void LikelihoodFieldModelPC::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_                = nh.param(param_name("max_beams"), 30);
    z_hit_                    = nh.param(param_name("z_hit"), 0.8);
    z_rand_                   = nh.param(param_name("z_rand"), 0.2);
}
}
