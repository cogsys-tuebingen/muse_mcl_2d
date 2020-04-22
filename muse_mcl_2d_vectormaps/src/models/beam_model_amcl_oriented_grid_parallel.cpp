#include "beam_model_amcl_oriented_grid_parallel.h"

#include <cslibs_plugins_data/types/laserscan.hpp>

#include <muse_mcl_2d_vectormaps/static_maps/oriented_grid_vectormap.h>
#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_vectormaps::BeamModelAMCLOrientedGridParallel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_vectormaps {
    BeamModelAMCLOrientedGridParallel::BeamModelAMCLOrientedGridParallel()
    {
    }

    void BeamModelAMCLOrientedGridParallel::apply(const data_t::ConstPtr             &data,
                                                  const state_space_t::ConstPtr      &map,
                                                  sample_set_t::weight_iterator_t     set)
    {
        if(!map->isType<static_maps::OrientedGridVectorMap>()) {
            return;
        }

        const laserscan_t &laser_data = data->as<laserscan_t>();

        data_ = data;
        map_ = map;

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

        rays_size_ = laser_data.getRays().size();
        rays_step_  = std::max(1ul, (rays_size_ - 1) / (max_beams_ - 1));
        range_max_  = laser_data.getLinearMax();
        p_rand_     = z_rand_ * 1.0 / range_max_;

        if(results_.size() == set.size()) {
            std::fill(results_.begin(), results_.end(), 0.0);
        } else {
            results_.resize(set.size(), 0.0);
        }

        /// enqueue
        std::size_t index = 0;
        for(auto it = set.const_begin() ; it != set.const_end() ; ++it, ++index) {
            samples_[index % 4].push(std::make_pair(&(it->state), index));
        }

        /// do work
        auto do_work = [this](const state_t &m_T_w,
                              const state_t &b_T_l,
                              std::queue<std::pair<state_t const*, std::size_t>> &q){
            const static_maps::OrientedGridVectorMap &vectormap = map_->as<static_maps::OrientedGridVectorMap>();
            const cslibs_vectormaps::OrientedGridVectorMap &map_data = vectormap.getMap();
            const laserscan_t &laser_data = data_->as<laserscan_t>();
            const laserscan_t::rays_t &rays = laser_data.getRays();

            while(!q.empty()) {
                std::pair<state_t const*, std::size_t> s = q.front();
                const state_t m_T_l = m_T_w * (*(s.first)) * b_T_l;
                results_[s.second] = probability(rays, m_T_l, map_data);
                q.pop();
            }
        };

        std::array<std::thread, 4> threads = {{std::thread([this, &m_T_w, &b_T_l, &do_work]{do_work(m_T_w, b_T_l, samples_[0]);}),
                                               std::thread([this, &m_T_w, &b_T_l, &do_work]{do_work(m_T_w, b_T_l, samples_[1]);}),
                                               std::thread([this, &m_T_w, &b_T_l, &do_work]{do_work(m_T_w, b_T_l, samples_[2]);}),
                                               std::thread([this, &m_T_w, &b_T_l, &do_work]{do_work(m_T_w, b_T_l, samples_[3]);})}};

        for(auto &t : threads) {
            if(t.joinable())
                t.join();
        }

        /// finalize
        auto p_it = results_.begin();
        const auto end = set.end();
        for(auto it = set.begin(); it != end; ++it, ++p_it) {
            *it *= *p_it;
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
