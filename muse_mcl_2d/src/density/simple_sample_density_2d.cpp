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
class SimpleSampleDensity2D : public muse_mcl_2d::SampleDensity2D
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

    using cis_kd_tree_buffered_t    = cis::Storage<sample_data_t, index_t, cis::backend::kdtree::KDTreeBuffered>;
    using cis_kd_tree_clustering_t  = cis::operations::clustering::Clustering<cis_kd_tree_buffered_t>;

    virtual void setup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const double resolution_linear          = nh.param<double>(param_name("resolution_linear"), 0.1);
        const double resolution_angular         = cslibs_math::common::angle::toRad(nh.param<double>(param_name("resolution_angular"), 5.0));
        const std::size_t maximum_sample_size   = static_cast<std::size_t>(nh.param<int>(param_name("maximum_sample_size"), 0));
        indexation_                             = {{resolution_linear, resolution_angular}};
        kdtree_.reset(new cis_kd_tree_buffered_t);
        kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * maximum_sample_size + 1);
        clustering_impl_                        = clustering_t(indexation_);
        ignore_weight_                          = nh.param<bool>(param_name("ignore_weight"), false);
    }

    virtual void clear() override
    {
        clustering_impl_.clear();
        kdtree_->clear();
        global_angle_.reset();
        global_position_.reset();
    }

    virtual void insert(const Sample2D &sample) override
    {
        kdtree_->insert(indexation_.create(sample), sample_data_t(sample, ignore_weight_));
        global_position_.add(sample.state.translation());
        global_angle_.add(sample.state.yaw());
    }

    virtual void estimate() override
    {
        cis_kd_tree_clustering_t clustering(*kdtree_);
        clustering.cluster(clustering_impl_);
    }

    cluster_map_t const & clusters() const override
    {
        return clustering_impl_.clusters;
    }

    distribution_map_t const & clusterDistributions() const override
    {
        return clustering_impl_.distributions;
    }

    angular_mean_map_t const & clusterAngularMeans() const override
    {
        return clustering_impl_.angular_means;
    }

    std::size_t histogramSize() const override
    {
        return kdtree_->size();
    }


    void mean(state_t &mean, covariance_t &covariance) const override
    {
        mean.translation() = global_position_.getMean();
        mean.setYaw(global_angle_.getMean());

        const Eigen::Matrix2d  linear_covariance    = global_position_.getCovariance();
        const double           angular_covariance   = global_angle_.getVariance();
        covariance(0,0) = linear_covariance(0,0);
        covariance(0,1) = linear_covariance(0,1);
        covariance(1,0) = linear_covariance(1,0);
        covariance(1,1) = linear_covariance(1,1);
        covariance(2,2) = angular_covariance;
    }

    bool maxClusterMean(state_t &mean, covariance_t &covariance) const
    {
        double max_weight = std::numeric_limits<double>::lowest();
        int    max_cluster_id = -1;


        for(const auto &cluster : clustering_impl_.clusters) {
            const int cluster_id = cluster.first;
            const auto &distribution = clustering_impl_.distributions.at(cluster_id);
            const auto weight = distribution.getWeight();
            if(weight > max_weight) {
                max_cluster_id = cluster_id;
                max_weight = weight;
            }
        }
        if(max_cluster_id != -1) {
            const auto &distribution = clustering_impl_.distributions.at(max_cluster_id);
            const auto &angular_mean = clustering_impl_.angular_means.at(max_cluster_id);
            mean.translation() = distribution.getMean();
            mean.setYaw(angular_mean.getMean());

            const Eigen::Matrix2d  linear_covariance    = distribution.getCovariance();
            const double           angular_covariance   = angular_mean.getCovariance();
            covariance(0,0) = linear_covariance(0,0);
            covariance(0,1) = linear_covariance(0,1);
            covariance(1,0) = linear_covariance(1,0);
            covariance(1,1) = linear_covariance(1,1);
            covariance(2,2) = angular_covariance;
            return true;
        }
        return false;
    }

protected:
    indexation_t                                indexation_;
    std::shared_ptr<cis_kd_tree_buffered_t>     kdtree_;

    clustering_t                                clustering_impl_;
    cslibs_math::statistics::Distribution<2,0>  global_position_;
    cslibs_math::statistics::AngularMean        global_angle_;
    bool                                        ignore_weight_;

};
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::SimpleSampleDensity2D, muse_mcl_2d::SampleDensity2D)
