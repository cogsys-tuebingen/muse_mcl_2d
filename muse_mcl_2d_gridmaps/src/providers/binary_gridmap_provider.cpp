#include "binary_gridmap_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BinaryGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
    BinaryGridmapProvider::BinaryGridmapProvider()
    {
    }

    BinaryGridmapProvider::state_space_t::ConstPtr BinaryGridmapProvider::getStateSpace() const
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if(!map_)
            notify_.wait(l);
        return map_;
    }

    void BinaryGridmapProvider::setup(ros::NodeHandle &nh)
    {
        auto param_name         = [this](const std::string &name){return name_ + "/" + name;};
        topic_                  = nh.param<std::string>(param_name("topic"), "/map");
        binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
        source_                 = nh.subscribe(topic_, 1, &BinaryGridmapProvider::callback, this);
        blocking_               = nh.param<bool>(param_name("blocking"), false);
    }

    void BinaryGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
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
                    cslibs_gridmaps::static_maps::BinaryGridmap::Ptr map;
                    cslibs_gridmaps::static_maps::conversion::from(*msg, map, binarization_threshold_);
                    std::unique_lock<std::mutex> l(map_mutex_);
                    map_.reset(new BinaryGridmap(map, msg->header.frame_id));
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
                    cslibs_gridmaps::static_maps::BinaryGridmap::Ptr map;
                    cslibs_gridmaps::static_maps::conversion::from(*msg, map, binarization_threshold_);
                    map_.reset(new BinaryGridmap(map, msg->header.frame_id));
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
