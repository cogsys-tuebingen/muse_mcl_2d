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

    using laserscan_t = cslibs_plugins_data::types::Laserscan2d;
    using transform_t = muse_mcl_2d::Sample2D::transform_t;
    using state_t     = muse_mcl_2d::Sample2D::state_t;
    using point_t     = muse_mcl_2d::Sample2D::state_space_boundary_t;

    const LikelihoodFieldGridmap::map_t &gridmap    = *(map->as<LikelihoodFieldGridmap>().data());
    const laserscan_t                   &laser_data = data->as<laserscan_t>();
    const laserscan_t::rays_t           &laser_rays = laser_data.getRays();

    /// laser to base transform
    transform_t b_T_l;
    transform_t m_T_w;
    if(!tf_->lookupTransform(robot_base_frame_,
                             laser_data.frame(),
                             ros::Time(laser_data.timeFrame().end.seconds()),
                             b_T_l,
                             tf_timeout_))
        return;
    if(!tf_->lookupTransform(world_frame_,
                             map->getFrame(),
                             ros::Time(laser_data.timeFrame().end.seconds()),
                             m_T_w,
                             tf_timeout_))
        return;

    const laserscan_t::rays_t rays = laser_data.getRays();
    const auto end = set.end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, (rays_size - 1) / (max_beams_ - 1));
    const double range_max      = laser_data.getLinearMax();
    const double p_rand         = z_rand_ * 1.0 / range_max;

    for(auto it = set.begin() ; it != end ; ++it) {
        const state_t m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            const point_t ray_end_point = m_T_l * ray.end_point;
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
