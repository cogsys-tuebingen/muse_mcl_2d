#ifndef MUSE_MCL_2D_GRIDMAPS_LASER_CONVEX_HULL_HPP
#define MUSE_MCL_2D_GRIDMAPS_LASER_CONVEX_HULL_HPP


#include <cslibs_plugins_data/types/laserscan.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)


namespace muse_mcl_2d_gridmaps {
namespace utilty {
namespace bg = boost::geometry;


struct Point
{
    Point(double x = 0, double y = 0, std::size_t index = 0) :
        x(x),
        y(y),
        index(index)
    {
    }

    Point(const Eigen::Vector2d &p, const std::size_t index) :
      x(p(0)),
      y(p(1)),
      index(index)
    {
    }

    double      x;
    double      y;
    std::size_t index;
} __attribute__ ((aligned (32)));

using polygon_t = boost::geometry::model::polygon<Point>;
using rays_t    = cslibs_plugins_data::types::Laserscan2::rays_t;
using ray_t     = cslibs_plugins_data::types::Laserscan2::Ray;
using laserscan_t = cslibs_plugins_data::types::Laserscan2;

inline void convexHull(const laserscan_t& scan,
                       std::vector<std::size_t> &indices)
{
    const rays_t rays = scan.getRays();
    const std::size_t size = rays.size();
    const double range_min = scan.getLinearMin();
    const double range_max = scan.getLinearMax();
    const double angle_min = scan.getAngularMin();
    const double angle_max = scan.getAngularMax();


    auto valid = [range_min, range_max, angle_min, angle_max](const ray_t &r){
        return r.valid()
            && r.angle >= angle_min && r.angle <= angle_max
            && r.range >= range_min && r.range <= range_max;
    };

    polygon_t scan_poly;
    polygon_t hull_poly;
    scan_poly.outer().reserve(size);
    for(std::size_t i = 0 ; i < size ; ++i) {
        const auto &r = rays[i];
        if(valid(r)) {
            scan_polygon.outer().emplace_back(Point(r.end_point(), i));
        }
    }
    boost::geometry::convex_hull(scan_poly, hull_poly);

    indices.reserve(hull_poly.outer().size());
    for(const auto &p : hull_poly.outer()) {
      indices.emplace_back(p.index);
    }
}
}

#endif // MUSE_MCL_2D_GRIDMAPS_LASER_CONVEX_HULL_HPP
