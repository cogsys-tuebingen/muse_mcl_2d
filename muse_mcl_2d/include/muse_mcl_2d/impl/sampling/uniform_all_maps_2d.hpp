#pragma once

#include <ros/time.h>

#include <cslibs_math/sampling/uniform.hpp>

#include <muse_mcl_2d/sampling/uniform_sampling_2d.hpp>
#include <cslibs_math_ros/tf/conversion_2d.hpp>

namespace muse_mcl_2d {


class EIGEN_ALIGN16 UniformAllMaps2D : public UniformSampling2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<UniformAllMaps2D>;
    using Metric = cslibs_math::sampling::Metric;
    using Radian = cslibs_math::sampling::Radian;
    using rng_t  = cslibs_math::sampling::Uniform<double, Metric, Metric, Radian>;

    using transform_t  = typename StateSpaceDescription2D::transform_t;

    virtual bool update(const std::string &frame) override;
    virtual void apply(Sample2D &sample) override;

protected:
    virtual bool apply(sample_set_t &particle_set) override;

    int random_seed_;
    std::vector<Map2D::ConstPtr>                       maps_;
    std::vector<transform_t, transform_t::allocator_t> maps_T_w_;
    std::vector<MapProvider2D::Ptr>                    map_providers_;
    rng_t::Ptr                                          rng_;

    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) override;
};
}
