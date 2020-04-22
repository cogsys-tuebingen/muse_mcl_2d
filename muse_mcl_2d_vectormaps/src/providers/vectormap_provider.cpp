#include "vectormap_provider.h"

#include <cslibs_vectormaps/loader/map_loader.hpp>

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_vectormaps::VectorMapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_vectormaps {
VectorMapProvider::state_space_t::ConstPtr VectorMapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    return map_;
}

void VectorMapProvider::waitForStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    while(!map_)
        notify_.wait(l);
}

void VectorMapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    map_file_ = nh.param<std::string>(param_name("map_file"), "");

    auto load = [this]()
    {
      cslibs_vectormaps::VectorMap::Ptr map;
      if (cslibs_vectormaps::MapLoader::load(map_file_, map)) {
         std::unique_lock<std::mutex> l(map_mutex_);
         map_.reset(new static_maps::VectorMap(map));
         l.unlock();
      } else
        throw std::runtime_error("Could not load file '" + map_file_ + "'!");

      notify_.notify_all();
    };

    worker_ = std::thread(load);
}
}
