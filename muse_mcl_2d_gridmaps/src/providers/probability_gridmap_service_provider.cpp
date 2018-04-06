#include "probability_gridmap_service_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProbabilityGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
ProbabilityGridmapServiceProvider::ProbabilityGridmapServiceProvider()
{
}

void ProbabilityGridmapServiceProvider::setup(ros::NodeHandle &nh_private)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    service_name_ = nh_private.param<std::string>(param_name("service"), "/static_map");
    source_ = nh_private.serviceClient<nav_msgs::GetMap>(service_name_);
    blocking_ = nh_private.param<bool>(param_name("blocking"), false);
}


ProbabilityGridmapServiceProvider::state_space_t::ConstPtr ProbabilityGridmapServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        auto load = [this, req]() {
            if(map_load_mutex_.try_lock()) {
                if(!map_ || cslibs_time::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
                    ROS_INFO_STREAM("[" << name_ << "]: Loading map.");
                    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr map;
                    cslibs_gridmaps::static_maps::conversion::from(req.response.map, map);
                    std::unique_lock<std::mutex> l(map_mutex_);
                    map_.reset(new ProbabilityGridmap(map, std::string(req.response.map.header.frame_id)));
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
                    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr map;
                    cslibs_gridmaps::static_maps::conversion::from(req.response.map, map);
                    map_.reset(new ProbabilityGridmap(map, req.response.map.header.frame_id));
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
