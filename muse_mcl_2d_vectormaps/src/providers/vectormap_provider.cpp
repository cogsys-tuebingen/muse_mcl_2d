#include "vectormap_provider.h"

#include <cslibs_vectormaps/loader/map_loader.hpp>

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_vectormaps::VectorMapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_vectormaps {
VectorMapProvider::VectorMapProvider()
{
}

VectorMapProvider::state_space_t::ConstPtr VectorMapProvider::getStateSpace() const
{
    return map_;
}

void VectorMapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    std::string map_file = nh.param<std::string>(param_name("map_file"), "");
    cslibs_vectormaps::VectorMap::Ptr map;

    if (!cslibs_vectormaps::MapLoader::load(map_file, map)) {
        ROS_ERROR_STREAM("Cannot load map \"" << map_file << "\"!");
        ros::shutdown();
        return;
    }

    map_.reset(new static_maps::VectorMap(map));
}
}
