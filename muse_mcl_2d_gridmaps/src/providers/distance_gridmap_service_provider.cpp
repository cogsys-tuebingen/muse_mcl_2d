#include "distance_gridmap_service_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_distance_gridmap.hpp>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::DistanceGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
    void DistanceGridmapServiceProvider::setup(ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        service_name_           = nh.param<std::string>(param_name("service"), "/static_map");
        binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
        maximum_distance_       = nh.param<double>(param_name("maximum_distance"), 2.0);
        source_                 = nh.serviceClient<nav_msgs::GetMap>(service_name_);
    }

    DistanceGridmapServiceProvider::state_space_t::ConstPtr DistanceGridmapServiceProvider::getStateSpace() const
    {
        if(!map_) {
            nav_msgs::GetMap req;
            if(source_.call(req)) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading map.");
                DistanceGridmap::map_t::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from<double,double>(req.response.map, map, binarization_threshold_, maximum_distance_);
                map_.reset(new DistanceGridmap(map, std::string(req.response.map.header.frame_id)));
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
            }
        }
        return map_;
    }
}
