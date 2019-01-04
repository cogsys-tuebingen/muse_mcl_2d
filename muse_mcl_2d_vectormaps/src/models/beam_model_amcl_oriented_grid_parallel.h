#pragma once

#include <muse_mcl_2d/update/update_model_2d.hpp>
#include <cmath>

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <muse_mcl_2d_vectormaps/static_maps/oriented_grid_vectormap.h>

#include <cslibs_utility/synchronized/synchronized_queue.hpp>

#include <deque>

namespace muse_mcl_2d_vectormaps {
class BeamModelAMCLOrientedGridParallel : public muse_mcl_2d::UpdateModel2D {
public:
    BeamModelAMCLOrientedGridParallel();

    virtual void apply(const data_t::ConstPtr          &data,
                       const state_space_t::ConstPtr   &map,
                       sample_set_t::weight_iterator_t set) override;

protected:
    std::size_t max_beams_;
    double      z_hit_;
    double      z_short_;
    double      z_max_;
    double      z_rand_;
    double      sigma_hit_;
    double      denominator_hit_;
    double      denominator_exponent_hit_;
    double      lambda_short_;
    double      chi_outlier_;

    double      p_rand_;
    double      range_max_;
    std::size_t rays_size_;
    std::size_t rays_step_;

    data_t::ConstPtr                                                data_;
    state_space_t::ConstPtr                                         map_;

    std::array<std::queue<std::pair<cslibs_math_2d::Pose2d const*, std::size_t>>, 4> samples_;

    std::vector<double>                                             results_;

    virtual void doSetup(ros::NodeHandle &nh) override;

    inline double pHit(const double ray_range, const double map_range) const
    {
        const double dz = ray_range - map_range;
        return z_hit_ * denominator_hit_ * std::exp(dz * dz * denominator_exponent_hit_);/// hit_sq_inv added

    }

    inline double pShort(const double ray_range, const double map_range) const
    {
        return ray_range <= map_range
                ? (z_short_ / (1.0 - std::exp(-lambda_short_ * map_range)) * lambda_short_  * std::exp((-lambda_short_ * ray_range)))
                : 0.0;
    }

    inline double pRand(const double ray_range) const
    {
            return ray_range < range_max_ ? p_rand_ : 0.0;
    }

    inline double pMax(const double ray_range) const
    {
        return ray_range >= range_max_ ? z_max_ : 0.0;
    }

    inline double pow3(const double x) const
    {
        return x*x*x;
    }

    inline double probability(const cslibs_plugins_data::types::Laserscan::rays_t  &rays,
                              const cslibs_math_2d::Pose2d                         &m_T_l,
                              const cslibs_vectormaps::OrientedGridVectorMap       &map_data) const
    {
            cslibs_vectormaps::VectorMap::Vector vectormap_ray;
            vectormap_ray.first.x(m_T_l.tx());
            vectormap_ray.first.y(m_T_l.ty());
            unsigned int vrow(0);
            unsigned int vcol(0);
            //const void* cell = cslibs_vectormap.cell(vectormap_ray.first);
            map_data.cellIndices(vectormap_ray.first, vrow, vcol);

            double p = 1.0;

            for(std::size_t i = 0; i < rays_size_; i += rays_step_) {
                const auto &ray = rays[i];

                /// <--- vectormap specific
                const double ray_angle = cslibs_math::common::angle::normalize(m_T_l.yaw() + ray.angle); // ray angle in map coordinates
                vectormap_ray.second.x(m_T_l.tx() + std::cos(ray_angle) * range_max_);
                vectormap_ray.second.y(m_T_l.ty() + std::sin(ray_angle) * range_max_);

                /// default range calculation
                const double map_range = map_data.intersectScanRay(vectormap_ray, vrow, vcol, ray_angle, range_max_);
                /// <--- vectormap specific

                const double ray_range = ray.range;
                p += ray.valid() ? pow3(pHit(ray_range, map_range) + pShort(ray_range, map_range) + pMax(ray_range) + pRand(ray_range)) :
                                   pow3(z_max_);
            }
            return p;
    }
};
}
