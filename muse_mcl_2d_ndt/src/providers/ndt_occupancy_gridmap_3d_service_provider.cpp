#include <muse_mcl_2d_ndt/providers/ndt_occupancy_gridmap_3d_service_provider.h>
#include <nav_msgs/GetMap.h>
#include <yaml-cpp/yaml.h>

#include <class_loader/register_macro.hpp>
#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <fstream>
CLASS_LOADER_REGISTER_CLASS(
    muse_mcl_2d_ndt::NDTOccupancyGridmap3dServiceProvider,
    muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
std::shared_ptr<NDTOccupancyGridmap3dServiceProvider::state_space_t const>
NDTOccupancyGridmap3dServiceProvider::getStateSpace() const {
  //    nav_msgs::GetMap req;
  //    if (source_.call(req))
  //        loadMap();
  throw std::runtime_error("Function not implemented!");
  return map_;
}

void NDTOccupancyGridmap3dServiceProvider::setup(ros::NodeHandle &nh) {
  auto param_name = [this](const std::string &name) {
    return name_ + "/" + name;
  };
  service_name_ = nh.param<std::string>(param_name("service"), "/static_map");
  source_ = nh.serviceClient<nav_msgs::GetMap>(service_name_);
}
}  // namespace muse_mcl_2d_ndt
