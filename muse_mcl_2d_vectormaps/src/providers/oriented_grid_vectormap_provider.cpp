#include "oriented_grid_vectormap_provider.h"

#include <cslibs_vectormaps/maps/oriented_grid_vector_map.h>
#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_vectormaps::OrientedGridVectorMapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_vectormaps {
OrientedGridVectorMapProvider::state_space_t::ConstPtr OrientedGridVectorMapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    return map_;
}

void OrientedGridVectorMapProvider::waitForStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    while(!map_)
        notify_.wait(l);
}

void OrientedGridVectorMapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    map_file_ = nh.param<std::string>(param_name("map_file"), "");

    auto load = [this]()
    {
        cslibs_vectormaps::OrientedGridVectorMap::Ptr map(new cslibs_vectormaps::OrientedGridVectorMap);
        if (map->load(map_file_)) {
            std::unique_lock<std::mutex> l(map_mutex_);
            map_.reset(new static_maps::OrientedGridVectorMap(map));
            l.unlock();
        } else
            throw std::runtime_error("Could not load file '" + map_file_ + "'!");

        notify_.notify_all();
    };

    worker_ = std::thread(load);
}
}
