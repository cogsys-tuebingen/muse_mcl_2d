#include <muse_mcl_2d_ndt/providers/ndt_gridmap_3d_provider.h>

#include <cslibs_ndt_3d/serialization/dynamic_maps/gridmap.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cslibs_ndt_3d/conversion/pointcloud.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::NDTGridmap3dProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
NDTGridmap3dProvider::NDTGridmap3dProvider()
{
}

NDTGridmap3dProvider::state_space_t::ConstPtr NDTGridmap3dProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    return map_;
}

void NDTGridmap3dProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_     = nh.param<std::string>(param_name("path"), "");
    frame_id_ = nh.param<std::string>(param_name("frame_id"), "/world");

    loadMap();
}

void NDTGridmap3dProvider::loadMap()
{    
    auto load = [this]() {
        std::unique_lock<std::mutex> l(map_mutex_);
        ROS_INFO_STREAM("Loading file '" << path_ << "'...");
        cslibs_ndt_3d::dynamic_maps::Gridmap::Ptr map;
        if (cslibs_ndt_3d::dynamic_maps::loadBinary(path_, map)) {
            map_.reset(new Gridmap3d(map, frame_id_));
            ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
        } else
            ROS_ERROR_STREAM("Could not load file '" << path_ << "'!");

        map_notify_.notify_one();
    };

    if(map_) {
        worker_ = std::thread(load);
    } else {
        load();
    }
}
}
