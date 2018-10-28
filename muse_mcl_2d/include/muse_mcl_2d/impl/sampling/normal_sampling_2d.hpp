#pragma once

#include <ros/time.h>

#include <cslibs_math/sampling/normal.hpp>

#include <muse_mcl_2d/sampling/normal_sampling_2d.hpp>
#include <cslibs_math_ros/tf/conversion_2d.hpp>

namespace muse_mcl_2d {

class EIGEN_ALIGN16 Normal2D : public NormalSampling2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Normal2D>;
    using Metric = cslibs_math::sampling::Metric;
    using Radian = cslibs_math::sampling::Radian;
    using rng_t  = cslibs_math::sampling::Normal<Metric, Metric, Radian>;

    virtual bool update(const std::string &frame) override;
    virtual bool apply(const cslibs_math_2d::Pose2d       &pose,
                       const cslibs_math_2d::Covariance3d &covariance,
                       sample_set_t &sample_set) override;

protected:
    int                                                                                random_seed_;
    std::vector<Map2D::ConstPtr>                                                       maps_;
    std::vector<cslibs_math_2d::Transform2d, cslibs_math_2d::Transform2d::allocator_t> maps_T_w_;
    std::vector<MapProvider2D::Ptr>                                                    map_providers_;

    virtual void doSetup(const std::map<std::string, MapProvider2D::Ptr> &map_providers,
                         ros::NodeHandle &nh) override;
};
}
