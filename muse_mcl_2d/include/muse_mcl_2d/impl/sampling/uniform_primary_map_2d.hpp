#pragma once

#include <ros/time.h>

#include <muse_mcl_2d/sampling/uniform_sampling_2d.hpp>


#include <cslibs_math/sampling/uniform.hpp>
#include <cslibs_math_ros/tf/conversion_2d.hpp>

namespace muse_mcl_2d {

class EIGEN_ALIGN16 UniformPrimaryMap2D : public UniformSampling2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<UniformPrimaryMap2D>;
    using Metric = cslibs_math::sampling::Metric;
    using Radian = cslibs_math::sampling::Radian;
    using rng_t  = cslibs_math::sampling::Uniform<Metric, Metric, Radian>;

    UniformPrimaryMap2D() = default;

    virtual bool update(const std::string &frame) override;
    virtual bool apply(sample_set_t &sample_set) override;
    virtual void apply(Sample2D &sample) override;

protected:
    std::vector<MapProvider2D::Ptr> map_providers_;
    int                             random_seed_;

    rng_t::Ptr                      rng_;
    cslibs_math_2d::Transform2d     w_T_primary_;
    Map2D::ConstPtr                 primary_map_;
    MapProvider2D::Ptr              primary_map_provider_;
    std::vector<Map2D::ConstPtr>    secondary_maps_;
    std::vector<cslibs_math_2d::Transform2d, cslibs_math_2d::Transform2d::allocator_t>  secondary_maps_T_w_;

    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) override;


};
}
