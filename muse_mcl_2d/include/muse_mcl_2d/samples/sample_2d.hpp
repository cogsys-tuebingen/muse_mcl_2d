#ifndef SAMPLE_2D_HPP
#define SAMPLE_2D_HPP

#include <memory>

#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_math_2d/linear/covariance.hpp>

namespace muse_mcl_2d {
struct Sample2D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Sample2D>;
    using Ptr         = std::shared_ptr<Sample2D>;
    using state_t     = cslibs_math_2d::Pose2d;

    double       weight;
    state_t      state;

    inline Sample2D() :
        weight(0.0),
        state(state_t(0.0,0.0))
    {
    }

    inline Sample2D(const Sample2D &other) :
        weight(other.weight),
        state(other.state)
    {
    }

    inline Sample2D(Sample2D &&other) :
        weight(other.weight),
        state(other.state)
    {
    }

    inline Sample2D& operator = (const Sample2D &other)
    {
        if(&other != this) {
            weight = other.weight;
            state  = other.state;
        }
        return *this;
    }

    inline Sample2D& operator = (Sample2D &&other)
    {
        if(&other != this) {
            weight = other.weight;
            state  = std::move(other.state);
        }
        return *this;
    }
};
}

#endif // SAMPLE_2D_HPP
