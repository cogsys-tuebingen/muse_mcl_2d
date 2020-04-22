#include "likelihood_field_gridmap_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_likelihood_field_gridmap.hpp>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
    LikelihoodFieldGridmapProvider::state_space_t::ConstPtr LikelihoodFieldGridmapProvider::getStateSpace() const
    {
        std::unique_lock<std::mutex> l(map_mutex_);        
        return map_;
    }

    void LikelihoodFieldGridmapProvider::waitForStateSpace() const
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if (!map_)
            notify_.wait(l);
    }

    void LikelihoodFieldGridmapProvider::setup(ros::NodeHandle &nh_private)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        topic_                  = nh_private.param<std::string>(param_name("topic"), "/map");
        binarization_threshold_ = nh_private.param<double>(param_name("threshold"), 0.5);
        maximum_distance_       = nh_private.param<double>(param_name("maximum_distance"), 2.0);
        z_hit_                  = nh_private.param<double>(param_name("z_hit"), 0.8);
        sigma_hit_              = nh_private.param<double>(param_name("sigma_hit"), 0.2);
        source_= nh_private.subscribe(topic_, 1, &LikelihoodFieldGridmapProvider::callback, this);
    }

    void LikelihoodFieldGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
    {
        if(!msg) {
            ROS_ERROR_STREAM("[" << name_ << "]: Received nullptr from ros!");
            return;
        }
        if(msg->info.height == 0 || msg->info.width == 0 || msg->info.resolution == 0) {
            ROS_ERROR_STREAM("[" << name_ << "]: Received empty map from ros!");
            return;
        }

        auto load = [this, msg]() {
            if(!map_ || cslibs_time::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading map [" << msg->info.width << " x " << msg->info.height << "]");
                LikelihoodFieldGridmap::map_t::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from<double,double>(*msg, map, maximum_distance_, sigma_hit_, binarization_threshold_);

                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new LikelihoodFieldGridmap(map, msg->header.frame_id));
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
                l.unlock();

                notify_.notify_all();
            }
        };

        worker_ = std::thread(load);
    }
}
