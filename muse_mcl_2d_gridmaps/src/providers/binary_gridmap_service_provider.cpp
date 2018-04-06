#include "binary_gridmap_service_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>

#include <nav_msgs/GetMap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BinaryGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
    BinaryGridmapServiceProvider::BinaryGridmapServiceProvider()
    {
    }

    void BinaryGridmapServiceProvider::setup(ros::NodeHandle &nh)
    {
        auto param_name         = [this](const std::string &name){return name_ + "/" + name;};
        service_name_           = nh.param<std::string>(param_name("service"), "/static_map");
        binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
        source_                 = nh.serviceClient<nav_msgs::GetMap>(service_name_);
        blocking_               = nh.param<bool>(param_name("blocking"), false);
    }


    BinaryGridmapServiceProvider::state_space_t::ConstPtr BinaryGridmapServiceProvider::getStateSpace() const
    {
        nav_msgs::GetMap req;
        if(source_.call(req)) {
            /// conversion can take time
            /// we allow concurrent loading, this way, the front end thread is not blocking.
            auto load = [this, req]() {
                if(map_load_mutex_.try_lock()) {
                    if(!map_ || cslibs_time::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
                        ROS_INFO_STREAM("[" << name_ << "]: Loading map.");
                        cslibs_gridmaps::static_maps::BinaryGridmap::Ptr map;
                        cslibs_gridmaps::static_maps::conversion::from(req.response.map, map, binarization_threshold_);
                        std::unique_lock<std::mutex> l(map_mutex_);
                        map_.reset(new BinaryGridmap(map, std::string(req.response.map.header.frame_id)));
                        ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
                    }
                    map_load_mutex_.unlock();
                    notify_.notify_one();
                }
            };
            auto load_blocking = [this, req]() {
                if(map_load_mutex_.try_lock()) {
                    if(!map_ || cslibs_time::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
                        std::unique_lock<std::mutex> l(map_mutex_);
                        ROS_INFO_STREAM("[" << name_ << "]: Loading map.");
                        cslibs_gridmaps::static_maps::BinaryGridmap::Ptr map;
                        cslibs_gridmaps::static_maps::conversion::from(req.response.map, map, binarization_threshold_);
                        map_.reset(new BinaryGridmap(map, req.response.map.header.frame_id));
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
        std::unique_lock<std::mutex> l(map_mutex_);
        if(!map_)
            notify_.wait(l);
        return map_;
    }
}
