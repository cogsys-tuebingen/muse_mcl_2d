#include "beam_model.h"


#include <cslibs_plugins_data/types/laserscan.hpp>
#include <muse_mcl_2d_gridmaps/maps/binary_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BeamModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_gridmaps {
BeamModel::BeamModel()
{
}

void BeamModel::apply(const data_t::ConstPtr          &data,
                      const state_space_t::ConstPtr   &map,
                      sample_set_t::weight_iterator_t set)
{
    if(!map->isType<BinaryGridmap>()) {
        return;
    }

    using laserscan_t = cslibs_plugins_data::types::Laserscan2d;
    using transform_t = muse_mcl_2d::StateSpaceDescription2D::transform_t;
    using state_t     = muse_mcl_2d::StateSpaceDescription2D::state_t;

    const BinaryGridmap::map_t &gridmap    = *(map->as<BinaryGridmap>().data());
    const laserscan_t          &laser_data = data->as<laserscan_t>();
    const laserscan_t::rays_t  &laser_rays = laser_data.getRays();

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
    const double range_max = laser_data.getLinearMax();
    const double p_rand = z_rand_ * 1.0 / range_max;


    /// mixture distribution entries
    auto pow2 = [](const double x) {return x*x;};
    auto p_hit = [this, &pow2](const double ray_range, const double map_range) {
        return z_hit_ * denominator_hit_ * std::exp(pow2(ray_range - map_range) * denominator_exponent_hit_);
    };
    auto p_short = [this](const double ray_range, const double map_range) {
        return ray_range < map_range ? z_short_ * lambda_short_ * std::exp(-lambda_short_ * ray_range) : 0.0;
        //return ray_range < map_range ? z_short_ * (1.0 / (1.0 - std::exp(-lambda_short_  * map_range))) * lambda_short_ * std::exp(-lambda_short_ * ray_range)
        //                               : 0.0;
    };
    auto p_max = [this, &range_max](const double ray_range) {
        return ray_range >= range_max ? z_max_ : 0.0;
    };
    auto p_random = [this, &range_max, &p_rand](const double ray_range) {
        return ray_range < range_max ? p_rand : 0.0;
    };
    auto probability = [&gridmap, &p_hit, &p_short, &p_max, &p_random, &range_max]
            (const laserscan_t::Ray &ray, const state_t &m_T_l) {
        const double ray_range = ray.range;
        auto         ray_end_point = m_T_l * ray.end_point;
        const double map_range = std::min(range_max, gridmap.getRange(m_T_l.translation(), ray_end_point));
        return p_hit(ray_range, map_range) + p_short(ray_range, map_range) + p_max(ray_range) + p_random(ray_range);
    };

    for(auto it = set.begin() ; it != end ; ++it) {
        const state_t m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;
        for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step) {
            const auto &ray = laser_rays[i];
            p *= ray.valid() ? probability(ray, m_T_l) : z_max_;
        }
        *it *= p;
    }
}

void BeamModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_    = nh.param(param_name("max_beams"), 30);
    z_hit_        = nh.param(param_name("z_hit"), 0.8);
    z_short_      = nh.param(param_name("z_short"), 0.1);
    z_max_        = nh.param(param_name("z_max"), 0.05);
    z_rand_       = nh.param(param_name("z_rand"), 0.05);
    sigma_hit_    = nh.param(param_name("sigma_hit"), 0.15);
    denominator_exponent_hit_   = -0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    denominator_hit_            = 1.0 / sqrt(2.0 * M_PI * sigma_hit_ * sigma_hit_);
    lambda_short_ = nh.param(param_name("lambda_short"), 0.01);
}
}
