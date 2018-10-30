#include "likelihood_field_prob_model_amcl.h"


#include <cslibs_plugins_data/types/laserscan.hpp>

#include <muse_mcl_2d_gridmaps/maps/distance_gridmap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldProbModelAMCL, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_gridmaps {
LikelihoodFieldProbModelAMCL::LikelihoodFieldProbModelAMCL()
{
}

void LikelihoodFieldProbModelAMCL::apply(const data_t::ConstPtr          &data,
                                         const state_space_t::ConstPtr   &map,
                                         sample_set_t::weight_iterator_t set)
{
    if(!map->isType<DistanceGridmap>()) {
        return;
    }

    const cslibs_gridmaps::static_maps::DistanceGridmap &gridmap = *(map->as<DistanceGridmap>().data());
    const cslibs_plugins_data::types::Laserscan                &laser_data = data->as<cslibs_plugins_data::types::Laserscan>();
    const cslibs_plugins_data::types::Laserscan::rays_t        &laser_rays = laser_data.getRays();

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
                             map->getFrame(),
                             ros::Time(laser_data.timeFrame().end.seconds()),
                             m_T_w,
                             tf_timeout_))
        return;

    const cslibs_plugins_data::types::Laserscan::rays_t rays = laser_data.getRays();
    const std::size_t rays_size = rays.size();
    const std::size_t ray_step  = std::max(1ul, (rays_size) / max_beams_);
    const double range_max = laser_data.getLinearMax();
    const double p_rand = z_rand_ * 1.0 / range_max;

    auto p_hit = [this] (const double z) {
        return z_hit_ * std::exp(-z * z * denominator_hit_);
    };


    std::fill(observation_histogram_.begin(), observation_histogram_.end(), 0);
    std::fill(observation_mask_.begin(), observation_mask_.end(), 0);
    const std::size_t sample_size = set.size();
    const std::size_t probability_buffer_size = sample_size * max_beams_;
    if(probability_buffer_size != observation_probability_buffer_.size()) {
        observation_probability_buffer_.resize(probability_buffer_size, 0.0);
    }
    std::fill(observation_probability_buffer_.begin(), observation_probability_buffer_.end(), 0.0);

    {   /// sensor probabilty calculation.
        const auto end = set.const_end();
        std::size_t sample_index = 0;
        for(auto it = set.const_begin() ; it != end ; ++it, ++sample_index) {
            const cslibs_math_2d::Pose2d m_T_l = m_T_w * it->state * b_T_l; /// laser scanner pose in map coordinates
            std::size_t observation_index = 0;
            for(std::size_t i = 0 ; i < rays_size ;  i+= ray_step, ++observation_index) {
                const auto &ray = laser_rays[i];
                if(!ray.valid())
                    continue;

                const cslibs_math_2d::Point2d ray_end_point = m_T_l * ray.end_point;
                const double distance = gridmap.at(ray_end_point);
                const double pz = p_hit(distance) + p_rand;

                if(distance < beam_skip_distance_) {
                    ++observation_histogram_[observation_index];
                }

                observation_probability_buffer_[sample_index * max_beams_ + observation_index] = pz;
            }
        }
    }
    {
        /// beam skipping
        std::size_t skipped_beam_count = 0;
        for(std::size_t i = 0 ; i < max_beams_ ; ++i) {
            if(observation_histogram_[i] / static_cast<double>(sample_size) > beam_skip_error_threshold_) {
                observation_mask_[i] = 1;
            } else {
                ++skipped_beam_count;
            }
        }

        bool error = skipped_beam_count >= max_beams_ * beam_skip_error_threshold_;
        std::size_t sample_index = 0;
        const auto end = set.end();
        for(auto it = set.begin() ; it != end ; ++it, ++sample_index) {
            double log_p = 0.0;
            for(std::size_t i = 0 ; i < max_beams_ ; ++i) {
                if(error || observation_mask_[i]) {
                    log_p += log(observation_probability_buffer_[sample_index * max_beams_ + i]);
                }
            }
            *it *= std::exp(log_p);
        }
    }
}

void LikelihoodFieldProbModelAMCL::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_beams_ = nh.param(param_name("max_beams"), 30);
    z_hit_ = nh.param(param_name("z_hit"), 0.8);
    z_rand_ = nh.param(param_name("z_rand"), 0.05);
    sigma_hit_ = nh.param(param_name("sigma_hit"), 0.15);
    denominator_hit_ = 0.5 * 1.0 / (sigma_hit_ * sigma_hit_);
    beam_skip_ = nh.param(param_name("beam_skip"), true);
    beam_skip_distance_ = nh.param(param_name("beam_skip_distance"), 0.5);
    beam_skip_threshold_= nh.param(param_name("beam_skip_threshold"),0.5 );
    beam_skip_error_threshold_= nh.param(param_name("beam_skip_error"), 0.5);

    observation_histogram_.resize(max_beams_, 0);
    observation_mask_.resize(max_beams_, 0);

}
}
