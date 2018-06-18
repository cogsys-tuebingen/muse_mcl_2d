#ifndef SAMPLE_CLUSTERING_2D_HPP
#define SAMPLE_CLUSTERING_2D_HPP

#include <unordered_map>

#include <cslibs_indexed_storage/operations/clustering.hpp>

#include "sample_density_data_2d.hpp"
#include "sample_indexation_2d.hpp"

#include <cslibs_math/common/mod.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d {
struct ClusterNeighbourhood2D {
    static constexpr std::size_t dimensions = 3;
    static constexpr std::size_t size       = 3;

    static constexpr std::size_t count = 10;

    using offset_value_t = typename boost::int_max_value_t<size>::fast;
    using offset_t       = std::array<offset_value_t, dimensions>;
    using offset_list_t  = std::array<offset_t, count>;

    template<typename visitor_t>
    static inline void visit(const visitor_t& visitor)
    {
        const static offset_list_t offsets = {{
            {-1,-1, 0},
            {-1, 0, 0},
            {-1, 1, 0},
            { 0,-1, 0},
            { 0, 0,-1},
            { 0, 0, 1},
            { 0, 1, 0},
            { 1,-1, 0},
            { 1, 0, 0},
            { 1, 1, 0} }};

        for (const auto& offset : offsets)
            visitor(offset);
    }
};


struct SampleClustering2D {
    using indexation_t          = SampleIndexation2D;
    using index_t               = SampleIndexation2D::index_t;
    using distribution_t        = SampleDensityData2D::distribution_t;
    using sample_ptr_vector_t   = SampleDensityData2D::sample_ptr_vector_t;
    using angular_mean_t        = SampleDensityData2D::angular_mean_t;

    using allocator_t           = Eigen::aligned_allocator<std::pair<const int, distribution_t>>;
    using distribution_map_t    = std::unordered_map<int,
    distribution_t,
    std::hash<int>,
    std::equal_to<int>,
    allocator_t>;
    using cluster_map_t         = std::unordered_map<int, sample_ptr_vector_t>;
    using angular_mean_map_t    = std::unordered_map<int, angular_mean_t>;

    /// required definitions -->
    using neighborhood_t        = ClusterNeighbourhood2D;
    using visitor_index_t       = neighborhood_t::offset_t;

    inline SampleClustering2D() = default;

    inline SampleClustering2D(indexation_t &indexation) :
        indexation(indexation)
    {
        auto resolution = indexation.getResolution();
        angular_bins = static_cast<int>(std::floor(2 * M_PI / resolution[1]));
    }

    inline void clear()
    {
        current_cluster = -1;
        clusters.clear();
        distributions.clear();
        angular_means.clear();
    }

    inline bool start(const index_t &, SampleDensityData2D& data)
    {
        if(data.cluster != -1)
            return false;

        ++current_cluster;

        clusters.emplace(current_cluster, data.samples);
        distributions.emplace(current_cluster, data.distribution);
        angular_means.emplace(current_cluster, data.angular_mean);

        data.cluster = current_cluster;

        return true;
    }

    inline bool extend(const index_t&, const index_t&, SampleDensityData2D& data)
    {
        if (data.cluster != -1)
            return false;

        auto& cluster      = clusters[current_cluster];
        auto& distribution = distributions[current_cluster];
        auto& angular_mean = angular_means[current_cluster];

        cluster.insert(cluster.end(), data.samples.begin(), data.samples.end());
        distribution += data.distribution;
        angular_mean += data.angular_mean;

        data.cluster = current_cluster;
        return true;
    }


    template<typename visitor_t>
    inline void visit_neighbours(const index_t&, const visitor_t& visitor)
    {
        static constexpr auto neighborhood = neighborhood_t{};
        neighborhood.visit(visitor);
    }

    template<typename offset_t>
    inline index_t add(const index_t& a, const offset_t& b) const
    {
        return index_t({a[0] + b[0],
                        a[1] + b[1],
                        cslibs_math::common::mod(a[2] + b[2], angular_bins)});
    }

    indexation_t       indexation;
    int                current_cluster = -1;
    int                angular_bins;
    cluster_map_t      clusters;
    distribution_map_t distributions;
    angular_mean_map_t angular_means;

};

}


#endif // SAMPLE_CLUSTERING_2D_HPP
