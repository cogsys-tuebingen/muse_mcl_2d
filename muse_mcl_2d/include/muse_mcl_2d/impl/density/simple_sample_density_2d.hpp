#pragma once

#include <ros/ros.h>
#include <unordered_map>

#include <muse_mcl_2d/density/sample_density_2d.hpp>

#include "sample_indexation_2d.hpp"
#include "sample_clustering_2d.hpp"

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>
#include <cslibs_indexed_storage/backend/simple/unordered_map.hpp>
#include <cslibs_indexed_storage/backend/simple/unordered_component_map.hpp>

#include <cslibs_math/statistics/distribution.hpp>
#include <cslibs_math/statistics/angular_mean.hpp>

namespace cis = cslibs_indexed_storage;

namespace std
{
//! needed for simple::UnorderedMap
template<>
struct hash<std::array<int, 3>>
{
    typedef std::array<int, 3> argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type const& s) const
    {
        result_type const h1 ( std::hash<int>{}(s[0]) );
        result_type const h2 ( std::hash<int>{}(s[1]) );
        result_type const h3 ( std::hash<int>{}(s[2]) );

        return (h1 ^ (h2 << 1)) | h3;
    }
};
}

namespace muse_mcl_2d {
class EIGEN_ALIGN16 SimpleSampleDensity2D : public muse_mcl_2d::SampleDensity2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<SimpleSampleDensity2D>;

    using indexation_t              = SampleIndexation2D;
    using sample_data_t             = SampleDensityData2D;
    using clustering_t              = SampleClustering2D;
    using distribution_t            = cslibs_math::statistics::WeightedDistribution<2,0>;
    using angular_mean_t            = cslibs_math::statistics::WeightedAngularMean;

    using index_t                   = indexation_t::index_t;

    using cluster_map_t             = clustering_t::cluster_map_t;
    using distribution_map_t        = clustering_t::distribution_map_t;
    using angular_mean_map_t        = clustering_t::angular_mean_map_t;

    /// using cis_kd_tree_buffered_t    = cis::Storage<sample_data_t, index_t, cis::backend::kdtree::KDTreeBuffered>;
    using cis_kd_tree_buffered_t    = cis::Storage<sample_data_t, index_t, cis::backend::simple::UnorderedComponentMap>;
    using cis_kd_tree_clustering_t  = cis::operations::clustering::Clustering<cis_kd_tree_buffered_t>;

    virtual void setup(ros::NodeHandle &nh) override;
    virtual void clear() override;
    virtual void insert(const Sample2D &sample) override;
    virtual void estimate() override;
    cluster_map_t const & clusters() const override;
    distribution_map_t const & clusterDistributions() const override;
    angular_mean_map_t const & clusterAngularMeans() const override;
    std::size_t histogramSize() const override;
    void mean(state_t &mean, covariance_t &covariance) const override;
    bool maxClusterMean(state_t &mean, covariance_t &covariance) const override;

protected:
    indexation_t                                indexation_;
    std::shared_ptr<cis_kd_tree_buffered_t>     kdtree_;

    clustering_t                                clustering_impl_;
    cslibs_math::statistics::Distribution<2,0>  global_position_;
    cslibs_math::statistics::AngularMean        global_angle_;
    bool                                        ignore_weight_;

};
}
