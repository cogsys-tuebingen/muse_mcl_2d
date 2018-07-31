#include <muse_mcl_2d_ndt/providers/binary_gridmap_provider.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/binary_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::BinaryGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
BinaryGridmapProvider::BinaryGridmapProvider()
{
}

BinaryGridmapProvider::state_space_t::ConstPtr BinaryGridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_)
        map_notify_.wait(l);
    return map_;
}

void BinaryGridmapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_                = nh.param<std::string>(param_name("path"), "");
    frame_id_            = nh.param<std::string>(param_name("frame_id"), "/world");

    sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), 0.05);
    threshold_           = nh.param<double>(param_name("threshold"), 0.196);

    loadMap();
}

void BinaryGridmapProvider::loadMap()
{
    auto load = [this]() {
        ROS_INFO_STREAM("Loading file '" << path_ << "'...");
        cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map;
        if (cslibs_ndt_2d::dynamic_maps::loadBinary(path_, map)) {

            cslibs_gridmaps::static_maps::BinaryGridmap::Ptr lf_map;
            cslibs_ndt_2d::conversion::from(map, lf_map, sampling_resolution_, threshold_);
            if (lf_map) {
                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new muse_mcl_2d_gridmaps::BinaryGridmap(lf_map, frame_id_));
                ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
            } else
                ROS_ERROR_STREAM("Could not convert map to Likelihood Field map");
        } else {
            ROS_ERROR_STREAM("Could not load file '" << path_ << "'!");
        }
        map_notify_.notify_all();
    };

    if(map_) {
        worker_ = std::thread(load);
    } else {
        load();
    }
}
}
