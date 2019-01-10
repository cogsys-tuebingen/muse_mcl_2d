#ifndef MUSE_MCL_2D_GRIDMAPS_MAP_LOADER_HPP
#define MUSE_MCL_2D_GRIDMAPS_MAP_LOADER_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include <ros/common.h>

namespace muse_mcl_2d_gridmaps {
namespace utility {
template<typename T>
void operator >> (const YAML::Node& node, T& i) {
    i = node.as<T>();
}

inline bool loadMap(const std::string &path,
                    const std::string &frame_id,
                    nav_msgs::OccupancyGrid::Ptr &dst)
{
    std::ifstream fin(path.c_str());
    if (fin.fail())
        return false;

    YAML::Node doc = YAML::Load(fin);
    std::string map_path = "";
    double origin[3];
    int negate;
    double occ_th, free_th, res;

#if ROS_VERSION >= 1060878
    MapMode mode = TRINARY;
#endif

    try {
        doc["resolution"] >> res;
    } catch (YAML::InvalidScalar &) { return false; }
    try {
        doc["negate"] >> negate;
    } catch (YAML::InvalidScalar &) { negate = 0; }
    try {
        doc["occupied_thresh"] >> occ_th;
    } catch (YAML::InvalidScalar &) { occ_th = 0.65; }
    try {
        doc["free_thresh"] >> free_th;
    } catch (YAML::InvalidScalar &) { free_th = 0.196; }

#if ROS_VERSION >= 1060878  /// KINETIC UBUNTU 16.04 (10.01.2019)
    try {
        std::string modeS = "";
        doc["mode"] >> modeS;

        if(modeS=="trinary")
            mode = TRINARY;
        else if(modeS=="scale")
            mode = SCALE;
        else if(modeS=="raw")
            mode = RAW;
        else
            return false;
    } catch (YAML::Exception &)  { mode = TRINARY; }
#endif
    try {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
    } catch (YAML::InvalidScalar &) { origin[0] = origin[1] = origin[2] = 0.0; }
    try {
        doc["image"] >> map_path;
        if(map_path.size() == 0)
            return false;
        if(map_path[0] != '/') {
            // dirname can modify what you pass it
            char* path_copy = strdup(path.c_str());
            map_path = std::string(dirname(path_copy)) + '/' + map_path;
            free(path_copy);
        }
    } catch (YAML::InvalidScalar &) { return false; }
    nav_msgs::GetMap::Response map_resp;
    try {
#if ROS_VERSION >= 1060878
        map_server::loadMapFromFile(&map_resp, map_path.c_str(), res, negate, occ_th, free_th, origin, mode);
#else
        map_server::loadMapFromFile(&map_resp, map_path.c_str(), res, negate, occ_th, free_th, origin);
#endif
    } catch (std::runtime_error &) { return false; }

    ros::Time::waitForValid();
    map_resp.map.header.frame_id    = frame_id;
    map_resp.map.info.map_load_time = ros::Time::now();
    map_resp.map.header.stamp       = ros::Time::now();

    dst.reset(new nav_msgs::OccupancyGrid());
    std::swap(*dst, map_resp.map);
    return true;
}
}
}

#endif // MUSE_MCL_2D_GRIDMAPS_MAP_LOADER_HPP
