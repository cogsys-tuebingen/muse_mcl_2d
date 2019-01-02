#include "beam_model_amcl_oriented_grid_parallel.h"

#include <cslibs_plugins_data/types/laserscan.hpp>

#include <muse_mcl_2d_vectormaps/static_maps/oriented_grid_vectormap.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>


#include <cslibs_utility/synchronized/synchronized_queue.hpp>

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_vectormaps::BeamModelAMCLOrientedGridParallel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_vectormaps {
    BeamModelAMCLOrientedGridParallel::BeamModelAMCLOrientedGridParallel()
    {
    }



    void BeamModelAMCLOrientedGridParallel::apply(const data_t::ConstPtr &data,
                                                const state_space_t::ConstPtr      &map,
                                                sample_set_t::weight_iterator_t     set)
    {
        if(!map->isType<static_maps::OrientedGridVectorMap>()) {
            return;
        }

        const static_maps::OrientedGridVectorMap &vectormap = map->as<static_maps::OrientedGridVectorMap>();
        const cslibs_vectormaps::OrientedGridVectorMap &map_data = vectormap.getMap();
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
        rays_size_ = rays.size();
        rays_step_  = std::max(1ul, (rays_size_ - 1) / (max_beams_ - 1));
        range_max_  = laser_data.getLinearMax();
        p_rand_     = z_rand_ * 1.0 / range_max_;

        for(auto it = set.begin(); it != end; ++it) {
            const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
            *it *= probability(laser_rays, m_T_l, map_data);
        }
    }

    void BeamModelAMCLOrientedGridParallel::doSetup(ros::NodeHandle &nh)
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
