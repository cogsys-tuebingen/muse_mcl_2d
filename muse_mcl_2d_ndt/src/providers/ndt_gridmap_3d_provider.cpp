#include <muse_mcl_2d_ndt/providers/ndt_gridmap_3d_provider.h>

#include <cslibs_ndt_3d/serialization/dynamic_maps/gridmap.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::NDTGridmap3dProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
NDTGridmap3dProvider::state_space_t::ConstPtr NDTGridmap3dProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    return map_;
}

void NDTGridmap3dProvider::waitForStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_)
        map_notify_.wait(l);
}

void NDTGridmap3dProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_     = nh.param<std::string>(param_name("path"), "");
    frame_id_ = nh.param<std::string>(param_name("frame_id"), "/world");

    auto load = [this]() {
        ROS_INFO_STREAM("Loading file '" << path_ << "'...");
        cslibs_ndt_3d::dynamic_maps::Gridmap<double>::Ptr map;
        if (cslibs_ndt_3d::dynamic_maps::loadBinary<double>(path_, map)) {
            std::unique_lock<std::mutex> l(map_mutex_);
            map_.reset(new Gridmap3d(map, frame_id_));
            ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
            l.unlock();
        } else
            throw std::runtime_error("Could not load file '" + path_ + "'!");
        map_notify_.notify_all();
    };

     worker_ = std::thread(load);
}
}
