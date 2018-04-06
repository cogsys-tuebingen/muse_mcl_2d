#include <muse_mcl_2d_ndt/providers/ndt_occupancy_gridmap_3d_service_provider.h>

#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_ndt_3d/conversion/pointcloud.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/GetMap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::NDTOccupancyGridmap3dServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
NDTOccupancyGridmap3dServiceProvider::NDTOccupancyGridmap3dServiceProvider()
{
}

NDTOccupancyGridmap3dServiceProvider::state_space_t::ConstPtr NDTOccupancyGridmap3dServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if (source_.call(req))
        loadMap();

    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_)
        map_notify_.wait(l);

    return map_;
}

void NDTOccupancyGridmap3dServiceProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    service_name_ = nh.param<std::string>(param_name("service"), "/static_map");
    path_         = nh.param<std::string>(param_name("path"), "");
    frame_id_     = nh.param<std::string>(param_name("frame_id"), "/world");

    source_ = nh.serviceClient<nav_msgs::GetMap>(service_name_);
}

void NDTOccupancyGridmap3dServiceProvider::loadMap() const
{
    auto load_blocking = [this]() {
        std::unique_lock<std::mutex> l(map_mutex_);
        ROS_INFO_STREAM("Loading file '" << path_ << "'...");
        cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::Ptr map;
        if (cslibs_ndt_3d::dynamic_maps::loadBinary(path_, map)) {
            map_.reset(new OccupancyGridmap3d(map, frame_id_));
            ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
        } else
            ROS_INFO_STREAM("Could not load file '" << path_ << "'!");

        map_notify_.notify_one();
    };

    worker_ = std::thread(load_blocking);
}
}
