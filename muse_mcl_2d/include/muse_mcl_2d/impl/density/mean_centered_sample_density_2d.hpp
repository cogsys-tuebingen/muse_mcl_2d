#pragma once

#include <ros/ros.h>
#include <unordered_map>

#include <muse_mcl_2d/density/sample_density_2d.hpp>
#include "sample_indexation_2d.hpp"
#include "sample_clustering_2d.hpp"

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree_buffered.hpp>

#include <cslibs_math/statistics/distribution.hpp>
#include <cslibs_math/statistics/angular_mean.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d {
/**
 * @brief The MCSampleDensity2D class is an extension to the simple density estimation using the last
 *        mean for the upcoming next iteration.
 */
class EIGEN_ALIGN16 MCSampleDensity2D : public muse_mcl_2d::SampleDensity2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MCSampleDensity2D>;

    using indexation_t              = SampleIndexation2D;
    using sample_data_t             = SampleDensityData2D;
    using clustering_t              = SampleClustering2D;

    using index_t                   = indexation_t::index_t;

    using cluster_map_t             = clustering_t::cluster_map_t;
    using distribution_t            = clustering_t::distribution_t;
    using angular_mean_t            = clustering_t::angular_mean_t;
    using distribution_map_t        = clustering_t::distribution_map_t;
    using angular_mean_map_t        = clustering_t::angular_mean_map_t;

    using cis_kd_tree_buffered_t    = cis::Storage<sample_data_t, index_t, cis::backend::kdtree::KDTreeBuffered>;
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

    mutable state_t                             offset_;
    clustering_t                                clustering_impl_;
    cslibs_math::statistics::Distribution<2,0>  global_position_;
    cslibs_math::statistics::AngularMean        global_angle_;
};

}
