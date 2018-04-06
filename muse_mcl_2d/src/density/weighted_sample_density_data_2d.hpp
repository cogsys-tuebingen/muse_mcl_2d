#ifndef WEIGHTED_SAMPLE_DENSITY_DATA_2D_HPP
#define WEIGHTED_SAMPLE_DENSITY_DATA_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>

#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <cslibs_math/statistics/weighted_angular_mean.hpp>

#include <vector>

namespace muse_mcl_2d {
struct WeightedSampleDensityData2D {
    using sample_ptr_vector_t = std::vector<const Sample2D *>;

    using distribution_t      = cslibs_math::statistics::WeightedDistribution<2>;
    using angular_mean_t      = cslibs_math::statistics::WeightedAngularMean;

    int                 cluster = -1;
    sample_ptr_vector_t samples;
    distribution_t      distribution;
    angular_mean_t      angular_mean;

    inline WeightedSampleDensityData2D()
    {
    }

    inline WeightedSampleDensityData2D(const WeightedSampleDensityData2D &other) :
        cluster(other.cluster),
        samples((other.samples)),
        distribution((other.distribution)),
        angular_mean((other.angular_mean))
    {
    }


    inline WeightedSampleDensityData2D(WeightedSampleDensityData2D &&other) :
        cluster(other.cluster),
        samples(std::move(other.samples)),
        distribution(std::move(other.distribution)),
        angular_mean(std::move(other.angular_mean))
    {
    }

    inline WeightedSampleDensityData2D& operator = (const WeightedSampleDensityData2D &other)
    {
        cluster         = (other.cluster);
        samples         = ((other.samples));
        distribution    = ((other.distribution));
        angular_mean    = ((other.angular_mean));
        return *this;
    }

    virtual ~WeightedSampleDensityData2D()
    {
    }


    inline WeightedSampleDensityData2D(const Sample2D &sample)
    {
        samples.emplace_back(&sample);
        distribution.add(sample.state.translation(), sample.weight);
        angular_mean.add(sample.state.yaw(), sample.weight);
     }

    inline void merge(const WeightedSampleDensityData2D &other)
    {
        samples.insert(samples.end(), other.samples.begin(), other.samples.end());
        distribution += other.distribution;
        angular_mean += other.angular_mean;
    }

};
}

#endif // WEIGHTED_SAMPLE_DENSITY_DATA_2D_HPP
