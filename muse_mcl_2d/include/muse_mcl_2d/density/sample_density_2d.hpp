#ifndef SAMPLE_DENSITY_2D_HPP
#define SAMPLE_DENSITY_2D_HPP

#include <ros/ros.h>
#include <unordered_map>
#include <class_loader/register_macro.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_smc/samples/sample_density.hpp>

#include <cslibs_math/statistics/weighted_angular_mean.hpp>
#include <cslibs_math/statistics/stable_weighted_distribution.hpp>

#include <cslibs_plugins/plugin.hpp>

namespace muse_mcl_2d {
class EIGEN_ALIGN16 SampleDensity2D : public muse_smc::SampleDensity<Sample2D>,
                                      public cslibs_plugins::Plugin
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr          = std::shared_ptr<SampleDensity2D>;
    using ConstPtr     = std::shared_ptr<SampleDensity2D const>;
    using state_t      = Sample2D::state_t;
    using covariance_t = muse_smc::traits::Covariance<Sample2D>::type;

    inline const static std::string Type()
    {
        return "muse_mcl_2d::SampleDensity2D";
    }

    using distribution_t      = cslibs_math::statistics::StableWeightedDistribution<double,2,0>;
    using angular_mean_t      = cslibs_math::statistics::WeightedAngularMean<double>;
    using distribution_map_t  = std::unordered_map<int,
                                                   distribution_t,
                                                   std::hash<int>,
                                                   std::equal_to<int>,
                                                   Eigen::aligned_allocator<std::pair<const int, distribution_t>>>;
    using angular_mean_map_t  = std::unordered_map<int,
                                                   angular_mean_t,
                                                   std::hash<int>,
                                                   std::equal_to<int>,
                                                   Eigen::aligned_allocator<std::pair<const int, angular_mean_t>>>;
    using sample_ptr_vector_t = std::vector<const Sample2D *>;
    using cluster_map_t       = std::unordered_map<int, sample_ptr_vector_t>;

    virtual void setup(ros::NodeHandle &nh) = 0;
    virtual cluster_map_t const & clusters() const = 0;
    virtual distribution_map_t const & clusterDistributions() const = 0;
    virtual angular_mean_map_t const & clusterAngularMeans() const = 0;
    virtual std::size_t histogramSize() const = 0;
    virtual bool maxClusterMean(state_t &mean, covariance_t &covariance) const = 0;
    virtual void mean(state_t &mean, covariance_t &covariance) const = 0;
};
}

#endif // SAMPLE_DENSITY_2D_HPP
