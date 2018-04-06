#include <muse_mcl_2d_ndt/providers/likelihood_field_gridmap_service_provider.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/likelihood_field_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_likelihood_field_gridmap.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/GetMap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::LikelihoodFieldGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
LikelihoodFieldGridmapServiceProvider::LikelihoodFieldGridmapServiceProvider()
{
}

LikelihoodFieldGridmapServiceProvider::state_space_t::ConstPtr LikelihoodFieldGridmapServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if (source_.call(req))
        loadMap();

    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_)
        map_notify_.wait(l);

    return map_;
}

void LikelihoodFieldGridmapServiceProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    service_name_        = nh.param<std::string>(param_name("service"), "/static_map");
    path_                = nh.param<std::string>(param_name("path"), "");
    frame_id_            = nh.param<std::string>(param_name("frame_id"), "/world");

    sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), 0.05);
    maximum_distance_    = nh.param<double>(param_name("maximum_distance"), 2.0);
    sigma_hit_           = nh.param<double>(param_name("sigma_hit"), 0.5);
    threshold_           = nh.param<double>(param_name("threshold"), 0.196);

    source_ = nh.serviceClient<nav_msgs::GetMap>(service_name_);
}

void LikelihoodFieldGridmapServiceProvider::loadMap() const
{    
    auto load_blocking = [this]() {
        std::unique_lock<std::mutex> l(map_mutex_);
        ROS_INFO_STREAM("Loading file '" << path_ << "'...");
        cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map;
        if (cslibs_ndt_2d::dynamic_maps::loadBinary(path_, map)) {

            cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr lf_map;
            cslibs_ndt_2d::conversion::from(map, lf_map, sampling_resolution_,
                                            maximum_distance_, sigma_hit_, threshold_);
            if (lf_map) {
                map_.reset(new muse_mcl_2d_gridmaps::LikelihoodFieldGridmap(lf_map, frame_id_));
                ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
            } else
                ROS_INFO_STREAM("Could not convert map to Likelihood Field map");
        } else
            ROS_INFO_STREAM("Could not load file '" << path_ << "'!");

        map_notify_.notify_one();
    };

    worker_ = std::thread(load_blocking);
}
}
