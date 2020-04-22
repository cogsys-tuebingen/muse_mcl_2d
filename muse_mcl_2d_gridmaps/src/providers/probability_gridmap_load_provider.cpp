#include "probability_gridmap_load_provider.h"

#include <muse_mcl_2d_gridmaps/utility/map_loader.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProbabilityGridmapLoadProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
    ProbabilityGridmapLoadProvider::state_space_t::ConstPtr ProbabilityGridmapLoadProvider::getStateSpace() const
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        return map_;
    }

    void ProbabilityGridmapLoadProvider::waitForStateSpace() const
    {
      std::unique_lock<std::mutex> l(map_mutex_);
      if (!map_)
          notify_.wait(l);
    }

    void ProbabilityGridmapLoadProvider::setup(ros::NodeHandle &nh)
    {
        auto param_name            = [this](const std::string &name){return name_ + "/" + name;};
        const std::string path     = nh.param<std::string>(param_name("path"), "");
        const std::string frame_id = nh.param<std::string>(param_name("frame_id"), "map");

        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        auto load = [this, &path, &frame_id]() {
            if (!map_) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading map [" << path << "]");
                nav_msgs::OccupancyGrid::Ptr msg;
                if (utility::loadMap(path, frame_id, msg)) {
                    ProbabilityGridmap::map_t::Ptr map;
                    cslibs_gridmaps::static_maps::conversion::from<double,double>(*msg, map);

                    std::unique_lock<std::mutex> l(map_mutex_);
                    map_.reset(new ProbabilityGridmap(map, msg->header.frame_id));
                    ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
                    l.unlock();

                    notify_.notify_all();
               } else
                    throw std::runtime_error("[" + name_ + "]: ERROR loading map.");
            }
        };

        worker_ = std::thread(load);
    }
}
