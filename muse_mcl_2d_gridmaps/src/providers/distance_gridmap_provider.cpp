#include "distance_gridmap_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_distance_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::DistanceGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
DistanceGridmapProvider::DistanceGridmapProvider()
{
}

DistanceGridmapProvider::state_space_t::ConstPtr DistanceGridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_)
        notify_.wait(l);
    return map_;
}

void DistanceGridmapProvider::setup(ros::NodeHandle &nh_private)
{
    auto param_name         = [this](const std::string &name){return name_ + "/" + name;};
    topic_                  = nh_private.param<std::string>(param_name("topic"), "/map");
    binarization_threshold_ = nh_private.param<double>(param_name("threshold"), 0.5);
    maximum_distance_       = nh_private.param<double>(param_name("maximum_distance"), 2.0);
    blocking_               = nh_private.param<bool>(param_name("blocking"), false);

    source_= nh_private.subscribe(topic_, 1, &DistanceGridmapProvider::callback, this);
}

void DistanceGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    if(!msg) {
        ROS_ERROR_STREAM("[" << name_ << "]: Received nullptr from ros!");
        return;
    }
    if(msg->info.height == 0 || msg->info.width == 0 || msg->info.resolution == 0) {
        ROS_ERROR_STREAM("[" << name_ << "]: Received empty map from ros!");
        return;
    }

    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    auto load = [this, msg]() {
        if(map_load_mutex_.try_lock()) {
            if(!map_ || cslibs_time::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading map [" << msg->info.width << " x " << msg->info.height << "]");
                cslibs_gridmaps::static_maps::DistanceGridmap::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from(*msg, map, binarization_threshold_, maximum_distance_);
                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new DistanceGridmap(map, msg->header.frame_id));
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
            }
            map_load_mutex_.unlock();
            notify_.notify_one();
        }
    };
    auto load_blocking = [this, msg]() {
        if(map_load_mutex_.try_lock()) {
            if(!map_ || cslibs_time::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
                std::unique_lock<std::mutex> l(map_mutex_);
                ROS_INFO_STREAM("[" << name_ << "]: Loading map [" << msg->info.width << " x " << msg->info.height << "]");
                cslibs_gridmaps::static_maps::DistanceGridmap::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from(*msg, map, binarization_threshold_, maximum_distance_);
                map_.reset(new DistanceGridmap(map, msg->header.frame_id));
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
            }
            map_load_mutex_.unlock();
            notify_.notify_one();
        }
    };
    if(blocking_)
        worker_ = std::thread(load_blocking);
    else
        worker_ = std::thread(load);
}
}
