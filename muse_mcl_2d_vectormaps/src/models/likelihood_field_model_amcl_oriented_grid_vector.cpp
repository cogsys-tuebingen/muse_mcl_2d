#include "likelihood_field_model_amcl_oriented_grid_vector.h"


#include <cslibs_plugins_data/types/laserscan.hpp>

#include <muse_mcl_2d_vectormaps/static_maps/oriented_grid_vectormap.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_vectormaps::LikelihoodFieldModelAMCLOrientedGridVector, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_vectormaps {
LikelihoodFieldModelAMCLOrientedGridVector::LikelihoodFieldModelAMCLOrientedGridVector()
{
}

void LikelihoodFieldModelAMCLOrientedGridVector::apply(const data_t::ConstPtr &data,
                                           const state_space_t::ConstPtr      &map,
                                           sample_set_t::weight_iterator_t     set)
{
    if(!map->isType<static_maps::OrientedGridVectorMap>()) {
        return;
    }

    const static_maps::OrientedGridVectorMap &vectormap = map->as<static_maps::OrientedGridVectorMap>();
    const cslibs_vectormaps::OrientedGridVectorMap &cslibs_vectormap = vectormap.getMap();
    const cslibs_plugins_data::types::Laserscan &laser_data = data->as<cslibs_plugins_data::types::Laserscan>();
    const cslibs_plugins_data::types::Laserscan::rays_t &laser_rays = laser_data.getRays();

    /// laser to base transform
    cslibs_math_2d::Transform2d b_T_l;
    cslibs_math_2d::Transform2d m_T_w;
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

    const cslibs_plugins_data::types::Laserscan::rays_t rays = laser_data.getRays();
    const auto end = set.end();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, (rays_size - 1) / (max_beams_ - 1));
    const double range_max = laser_data.getLinearMax();
    const double _1_z_max = 1.0 / range_max;
    const double p_rand = z_rand_ * _1_z_max;

    auto p_hit = [this, &_1_z_max, &p_rand](const double z) {
        return (std::fabs(z) > max_occ_dist_ || z == std::numeric_limits<double>::max()) ?
                    _1_z_max :
                    z_hit_ * denominator_hit_ * std::exp(z*z * denominator_exponent_hit_) + p_rand;
        //return z_hit_ * denominator_hit_ * std::exp(zz * denominator_exponent_hit_);
    };

    for(auto it = set.begin(); it != end; ++it) {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
        double p = 1.0;

        /// <--- vectormap specific
        //const void* cell = cslibs_vectormap.cell({m_T_l.tx(), m_T_l.ty()});
        unsigned int vrow(0);
        unsigned int vcol(0);
        cslibs_vectormap.cellIndices(m_T_l.tx(), m_T_l.ty(), vrow, vcol);
        /// <--- vectormap specific

        for(std::size_t i = 0; i < rays_size; i += ray_step) {
            const auto &ray = laser_rays[i];
            if(!ray.valid())
                continue;

            const cslibs_math_2d::Pose2d ray_end_point = m_T_l * ray.end_point;

            /// <--- vectormap specific
            const double ray_angle = m_T_l.yaw() + ray.angle; // ray angle in map coordinates
            const cslibs_vectormaps::VectorMap::Point vp(ray_end_point.tx(), ray_end_point.ty());
            //const double zz = cslibs_vectormap.minSquaredDistanceNearbyStructure(vp, cell, ray_angle);
            const double z = cslibs_vectormap.minDistanceNearbyStructure(vp, vrow, vcol, ray_angle);
            /// <--- vectormap specific

            const double pz = p_hit(z);// + p_rand;
            p += pz * pz * pz;
        }
        *it *= p;
    }
}

void LikelihoodFieldModelAMCLOrientedGridVector::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_ = nh.param(param_name("max_beams"), 30);
    z_hit_     = nh.param(param_name("z_hit"), 0.8);
    z_rand_    = nh.param(param_name("z_rand"), 0.2);
    sigma_hit_ = nh.param(param_name("sigma_hit"), 0.15);
    denominator_exponent_hit_ = -0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    denominator_hit_          = 1.0 / (std::sqrt(2.0 * M_PI) * sigma_hit_);
    max_occ_dist_             = nh.param(param_name("max_dist"), 2.0);
}
}
