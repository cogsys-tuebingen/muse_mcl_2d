#include "likelihood_field_model_log.h"


#include <cslibs_plugins_data/types/laserscan.hpp>
#include <muse_mcl_2d_gridmaps/maps/distance_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldModelLog, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_gridmaps {
LikelihoodFieldModelLog::LikelihoodFieldModelLog()
{
}

void LikelihoodFieldModelLog::apply(const data_t::ConstPtr          &data,
                                 const state_space_t::ConstPtr   &map,
                                 sample_set_t::weight_iterator_t set)
{
    if(!map->isType<DistanceGridmap>()) {
        return;
    }
    if(ps_.size() != set.capacity()) {
        ps_.resize(set.capacity(), 0.0);
    }
    std::fill(ps_.begin(), ps_.end(), 0.0);

    using laserscan_t = cslibs_plugins_data::types::Laserscan2d;

    const DistanceGridmap::map_t &gridmap    = *(map->as<DistanceGridmap>().data());
    const laserscan_t            &laser_data = data->as<laserscan_t>();
    const laserscan_t::rays_t    &laser_rays = laser_data.getRays();

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
    const auto const_end = set.const_end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, (rays_size - 1) / (max_beams_ - 1));
    const double range_max      = laser_data.getLinearMax();
    const double p_rand         = z_rand_ * 1.0 / range_max;

    auto p_hit = [this] (const double dz) {
        return z_hit_ * denominator_hit_ * std::exp(dz * dz * denominator_exponent_hit_);
    };

    auto it_ps = ps_.begin();
    double log_max = std::numeric_limits<double>::lowest();
    for(auto it = set.const_begin() ; it != const_end ; ++it, ++it_ps) {
        const state_t m_T_l = m_T_w * it->state * b_T_l; /// laser scanner pose in map coordinates
        double p = 0.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            const point_t   ray_end_point = m_T_l * ray.end_point;
            const double pz = ray.valid() ? p_hit(gridmap.at(ray_end_point)) + p_rand : 0.0;
            p += std::log(pz);
        }
        *it_ps = p;
        log_max = p >= log_max ? p : log_max;
    }

    it_ps = ps_.begin();
    for(auto it = set.begin() ; it != end ; ++it, ++it_ps) {
        *it *= std::exp(*it_ps - log_max);
    }
}

void LikelihoodFieldModelLog::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_                = nh.param(param_name("max_beams"), 30);
    z_hit_                    = nh.param(param_name("z_hit"), 0.8);
    z_rand_                   = nh.param(param_name("z_rand"), 0.2);
    sigma_hit_                = nh.param(param_name("sigma_hit"), 0.15);
    denominator_exponent_hit_ = -0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    denominator_hit_          = 1.0 / sqrt(2.0 * M_PI * sigma_hit_ * sigma_hit_);
}
}
