#include <muse_mcl_2d_ndt/providers/probability_occupancy_gridmap_provider.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::ProbabilityOccupancyGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
std::shared_ptr<ProbabilityOccupancyGridmapProvider::state_space_t const> ProbabilityOccupancyGridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    return map_;
}

void ProbabilityOccupancyGridmapProvider::waitForStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_)
        map_notify_.wait(l);
}

void ProbabilityOccupancyGridmapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_                = nh.param<std::string>(param_name("path"), "");
    frame_id_            = nh.param<std::string>(param_name("frame_id"), "/world");
    sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), 0.05);

    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel<double>(prob_prior, prob_free, prob_occupied));

    auto load_blocking = [this]() {
        ROS_INFO_STREAM("Loading file '" << path_ << "'...");
        cslibs_ndt_2d::dynamic_maps::OccupancyGridmap<double>::Ptr map;
        if (cslibs_ndt_2d::dynamic_maps::loadBinary<double>(path_, map)) {

            cslibs_gridmaps::static_maps::ProbabilityGridmap<double,double>::Ptr lf_map;
            cslibs_ndt_2d::conversion::from<double>(map, lf_map, sampling_resolution_, inverse_model_);
            if (lf_map) {
                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new muse_mcl_2d_gridmaps::ProbabilityGridmap(lf_map, frame_id_));
                ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
                l.unlock();
            } else
                ROS_INFO_STREAM("Could not convert map to Likelihood Field map");
        } else
            ROS_INFO_STREAM("Could not load file '" << path_ << "'!");
        map_notify_.notify_all();
    };

    worker_ = std::thread(load_blocking);
}
}
