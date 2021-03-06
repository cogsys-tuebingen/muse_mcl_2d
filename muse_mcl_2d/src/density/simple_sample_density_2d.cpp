#include <muse_mcl_2d/impl/density/simple_sample_density_2d.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d {

void SimpleSampleDensity2D::setup(ros::NodeHandle& nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const double resolution_linear          = nh.param<double>(param_name("resolution_linear"), 0.1);
    const double resolution_angular         = cslibs_math::common::angle::toRad(nh.param<double>(param_name("resolution_angular"), 5.0));
    const std::size_t maximum_sample_size   = static_cast<std::size_t>(nh.param<int>(param_name("maximum_sample_size"), 0));
    indexation_                             = {{resolution_linear, resolution_angular}};
    kdtree_.reset(new cis_kd_tree_buffered_t);
    kdtree_->set<cis::option::tags::node_allocator_chunk_size>(2 * maximum_sample_size + 1);
    clustering_impl_                        = clustering_t(indexation_);
}

void SimpleSampleDensity2D::clear()
{
    clustering_impl_.clear();
    kdtree_->clear();
    global_angle_.reset();
    global_position_.reset();
}

void SimpleSampleDensity2D::insert(const Sample2D& sample)
{
    kdtree_->insert(indexation_.create(sample), sample_data_t(sample));
    global_position_.add(sample.state().translation());
    global_angle_.add(sample.state().yaw());
}

void SimpleSampleDensity2D::estimate()
{
    cis_kd_tree_clustering_t clustering(*kdtree_);
    clustering.cluster(clustering_impl_);
}

SimpleSampleDensity2D::cluster_map_t const& SimpleSampleDensity2D::clusters() const
{
    return clustering_impl_.clusters;
}

const SimpleSampleDensity2D::distribution_map_t& SimpleSampleDensity2D::clusterDistributions() const
{
    return clustering_impl_.distributions;
}

const SimpleSampleDensity2D::angular_mean_map_t& SimpleSampleDensity2D::clusterAngularMeans() const
{
    return clustering_impl_.angular_means;
}

std::size_t SimpleSampleDensity2D::histogramSize() const
{
    return kdtree_->size();
}

void SimpleSampleDensity2D::mean(state_t& mean, covariance_t& covariance) const
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

bool SimpleSampleDensity2D::maxClusterMean(state_t& mean, covariance_t& covariance) const
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
}

CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d::SimpleSampleDensity2D, muse_mcl_2d::SampleDensity2D)
