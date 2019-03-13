#ifndef MUSE_MCL_2D_NDT_LASER_HISTOGRAM_HPP
#define MUSE_MCL_2D_NDT_LASER_HISTOGRAM_HPP

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree.hpp>
#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_math/statistics/distribution.hpp>


namespace muse_mcl_2d_ndt {
namespace utility_ray {
namespace cis        = cslibs_indexed_storage;
using laserscan_t    = cslibs_plugins_data::types::Laserscan2d;
using ray_t          = laserscan_t::Ray;
using rays_t         = laserscan_t::rays_t;
using point_t        = laserscan_t::point_t;
using distribution_t = cslibs_math::statistics::Distribution<double,2>;
using index_t        = std::array<int, 2>;

struct EIGEN_ALIGN16 Data {
    std::vector<size_t> indices;
    distribution_t      distribution;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Data() = default;

    inline Data(const std::size_t ri,
                const ray_t &r) :
        indices(1, ri)
    {
        distribution += r.end_point;
    }

    inline void merge (const Data &other)
    {
        distribution += other.distribution;
        indices.insert(indices.end(), other.indices.begin(), other.indices.end());
    }
};

struct Indexation {
    double resolution;

    Indexation(const double resolution = 0.1) :
        resolution(resolution)
    {
    }

    inline index_t create(const ray_t &r)
    {
        return {{static_cast<int>(r.end_point(0) / resolution),
                 static_cast<int>(r.end_point(1) / resolution)}};
    }
};

using kd_tree_t = cis::Storage<Data, index_t, cis::backend::kdtree::KDTree>;

inline void getMeans(const kd_tree_t &histogram,
                     std::vector<point_t, point_t::allocator_t> &points)
{
    auto traverse = [&points](const index_t &, const Data &d) {
        points.emplace_back(d.distribution.getMean());
    };
    histogram.traverse(traverse);
}

inline void getRepresentativeRays(const kd_tree_t &histogram,
                                  const rays_t &rays,
                                  std::vector<std::size_t> &indices)
{
    auto traverse = [&indices, &rays](const index_t &, const Data &d) {
        double min_distance = std::numeric_limits<double>::max();
        std::size_t index = 0;
        const point_t m = point_t(d.distribution.getMean());
        for (const std::size_t i : d.indices) {
            const double distance = cslibs_math::linear::distance2(rays.at(i).end_point, m);
            if (distance < min_distance) {
                min_distance = distance;
                index = i;
            }
        }
        if (min_distance < std::numeric_limits<double>::max())
            indices.emplace_back(index);
    };
    histogram.traverse(traverse);
}
}
}

#endif // MUSE_MCL_2D_NDT_LASER_HISTOGRAM_HPP
