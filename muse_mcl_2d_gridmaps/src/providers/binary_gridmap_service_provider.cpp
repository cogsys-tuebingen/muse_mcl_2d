#include "binary_gridmap_service_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>

#include <nav_msgs/GetMap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BinaryGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
    void BinaryGridmapServiceProvider::setup(ros::NodeHandle &nh)
    {
        auto param_name         = [this](const std::string &name){return name_ + "/" + name;};
        service_name_           = nh.param<std::string>(param_name("service"), "/static_map");
        binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
        source_                 = nh.serviceClient<nav_msgs::GetMap>(service_name_);
    }


    BinaryGridmapServiceProvider::state_space_t::ConstPtr BinaryGridmapServiceProvider::getStateSpace() const
    {
        if(!map_) {
            nav_msgs::GetMap req;
            if(source_.call(req)) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading map.");
                BinaryGridmap::map_t::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from<double>(req.response.map, map, binarization_threshold_);
                map_.reset(new BinaryGridmap(map, std::string(req.response.map.header.frame_id)));
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
            }
        }
        return map_;
    }
}
