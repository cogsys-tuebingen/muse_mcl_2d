#include "likelihood_field_gridmap_load_provider.h"

#include <class_loader/register_macro.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_likelihood_field_gridmap.hpp>
#include <muse_mcl_2d_gridmaps/utility/map_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(
    muse_mcl_2d_gridmaps::LikelihoodFieldGridmapLoadProvider,
    muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
std::shared_ptr<LikelihoodFieldGridmapLoadProvider::state_space_t const>
LikelihoodFieldGridmapLoadProvider::getStateSpace() const {
  std::unique_lock<std::mutex> l(map_mutex_);
  return map_;
}

void LikelihoodFieldGridmapLoadProvider::waitForStateSpace() const {
  std::unique_lock<std::mutex> l(map_mutex_);
  if (!map_) notify_.wait(l);
}

void LikelihoodFieldGridmapLoadProvider::setup(ros::NodeHandle &nh) {
  auto param_name = [this](const std::string &name) {
    return name_ + "/" + name;
  };
  const std::string path = nh.param<std::string>(param_name("path"), "");
  const std::string frame_id =
      nh.param<std::string>(param_name("frame_id"), "map");

  binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
  maximum_distance_ = nh.param<double>(param_name("maximum_distance"), 2.0);
  z_hit_ = nh.param<double>(param_name("z_hit"), 0.8);
  sigma_hit_ = nh.param<double>(param_name("sigma_hit"), 0.2);

  /// conversion can take time
  /// we allow concurrent loading, this way, the front end thread is not
  /// blocking.
  auto load = [this, &path, &frame_id]() {
    if (!map_) {
      ROS_INFO_STREAM("[" << name_ << "]: Loading map [" << path << "]");
      nav_msgs::OccupancyGrid::Ptr msg;
      if (utility::loadMap(path, frame_id, msg)) {
        LikelihoodFieldGridmap::map_t::Ptr map;
        cslibs_gridmaps::static_maps::conversion::from<double, double>(
            *msg, map, maximum_distance_, sigma_hit_, binarization_threshold_);

        std::unique_lock<std::mutex> l(map_mutex_);
        map_.reset(new LikelihoodFieldGridmap(map, msg->header.frame_id));
        ROS_INFO_STREAM("[" << name_ << "]: Loaded map.");
        l.unlock();

        notify_.notify_all();
      } else
        throw std::runtime_error("[" + name_ + "]: ERROR loading map.");
    }
  };

  worker_ = std::thread(load);
}
}  // namespace muse_mcl_2d_gridmaps
