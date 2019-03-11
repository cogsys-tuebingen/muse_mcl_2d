#include "beam_model_amcl_vector.h"


#include <cslibs_plugins_data/types/laserscan.hpp>

#include <muse_mcl_2d_vectormaps/static_maps/vectormap.h>
#include <cslibs_vectormaps/maps/vector_map.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>
#include <cmath>

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_vectormaps::BeamModelAMCLVector, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_vectormaps {
BeamModelAMCLVector::BeamModelAMCLVector()
{
}

void BeamModelAMCLVector::apply(const data_t::ConstPtr          &data,
                                const state_space_t::ConstPtr   &map,
                                sample_set_t::weight_iterator_t set)
{
    if(!map->isType<static_maps::VectorMap>()) {
        return;
    }

    using laserscan_t = cslibs_plugins_data::types::Laserscan<double>;
    const static_maps::VectorMap &vectormap = map->as<static_maps::VectorMap>();
    const cslibs_vectormaps::VectorMap &cslibs_vectormap = vectormap.getMap();
    const laserscan_t &laser_data = data->as<laserscan_t>();
    const laserscan_t::rays_t &laser_rays = laser_data.getRays();

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
                             vectormap.getFrame(),
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
    auto pow3   = [](const double x) {return x*x*x;};

    /// mixture distribution entries
    auto p_hit = [this](const double ray_range, const double map_range) {
        const double dz = ray_range - map_range;
        return z_hit_ * denominator_hit_ * std::exp(dz * dz * denominator_exponent_hit_);/// hit_sq_inv added
    };
    auto p_short = [this](const double ray_range, const double map_range) {
        return ray_range <= map_range
            ? z_short_ * /*(1.0 / (1.0 - std::exp(-lambda_short_ * map_range))) * lambda_short_ **/ std::exp(-lambda_short_ * ray_range) / std::exp(-lambda_short_ * map_range)
            : 0.0;
    };
    auto p_max = [this, range_max](const double ray_range)
    {
        return ray_range >= range_max ? z_max_ : 0.0;
    };
    auto p_random = [this, range_max, p_rand](const double ray_range)
    {
        return ray_range < range_max ? p_rand : 0.0;
    };
    auto probability = [&cslibs_vectormap, &p_hit, &p_short, &p_max, &p_random, range_max]
            (const laserscan_t::Ray &ray, const state_t &m_T_l,
            cslibs_vectormaps::VectorMap::Vector &vectormap_ray, const void* cell)
    {
        /// <--- vectormap specific
        const double ray_angle = m_T_l.yaw() + ray.angle; // ray angle in map coordinates
        vectormap_ray.second.x(m_T_l.tx() + std::cos(ray_angle) * range_max);
        vectormap_ray.second.y(m_T_l.ty() + std::sin(ray_angle) * range_max);
        /// default range calculation
        const double map_range = cslibs_vectormap.intersectScanRay(
                    vectormap_ray, cell, ray_angle, range_max);
        /// <--- vectormap specific

        const double ray_range = ray.range;
        return p_hit(ray_range, map_range) + p_short(ray_range, map_range) + p_max(ray_range) + p_random(ray_range);
    };

    for(auto it = set.begin(); it != end; ++it) {
        const state_t m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;

        /// <--- vectormap specific
        cslibs_vectormaps::VectorMap::Vector vectormap_ray;
        vectormap_ray.first.x(m_T_l.tx());
        vectormap_ray.first.y(m_T_l.ty());
        const void* cell = cslibs_vectormap.cell(vectormap_ray.first);
        /// <--- vectormap specific

        for(std::size_t i = 0; i < rays_size; i += ray_step) {
            const auto &ray = laser_rays[i];
            p += ray.valid() ? pow3(probability(ray, m_T_l, vectormap_ray, cell)) : pow3(z_max_);
        }
        *it *= p;
    }
}

void BeamModelAMCLVector::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_    = nh.param(param_name("max_beams"), 30);
    z_hit_        = nh.param(param_name("z_hit"), 0.8);
    z_short_      = nh.param(param_name("z_short"), 0.1);
    z_max_        = nh.param(param_name("z_max"), 0.05);
    z_rand_       = nh.param(param_name("z_rand"), 0.05);
    sigma_hit_    = nh.param(param_name("sigma_hit"), 0.15);
    denominator_exponent_hit_ = -0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    denominator_hit_          = 1.0 / (std::sqrt(2.0 * M_PI) * sigma_hit_);
    lambda_short_ = nh.param(param_name("lambda_short"), 0.01);
    chi_outlier_  = nh.param(param_name("chi_outlier"), 0.05);
}
}
