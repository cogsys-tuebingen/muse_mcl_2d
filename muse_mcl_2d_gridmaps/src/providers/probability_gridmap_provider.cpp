#include "probability_gridmap_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProbabilityGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
    ProbabilityGridmapProvider::state_space_t::ConstPtr ProbabilityGridmapProvider::getStateSpace() const
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        return map_;
    }

    void ProbabilityGridmapProvider::waitForStateSpace() const
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if (!map_)
            notify_.wait(l);
    }

    void ProbabilityGridmapProvider::setup(ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        topic_          = nh.param<std::string>(param_name("topic"), "/map");
        source_         = nh.subscribe(topic_, 1, &ProbabilityGridmapProvider::callback, this);
    }

    void ProbabilityGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
    {
        if(!msg) {
            ROS_ERROR_STREAM("[" << name_ << "]: Received nullptr from ros!");
            return;
        }
        if(msg->info.height == 0 || msg->info.width == 0 || msg->info.resolution == 0) {
            ROS_ERROR_STREAM("[" << name_ << "]: Received empty map from ros!");
            return;
        }

        auto load = [this, msg]() {
            if(!map_ || cslibs_time::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
                ROS_INFO_STREAM("[" << name_ << "]: Loading map [" << msg->info.width << " x " << msg->info.height << "]");
                ProbabilityGridmap::map_t::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from<double,double>(*msg, map);

                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new ProbabilityGridmap(map, msg->header.frame_id));
                ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
                l.unlock();

                notify_.notify_all();
            }
        };

        worker_ = std::thread(load);
    }
}
