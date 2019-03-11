#ifndef STATE_SPACE_2D_HPP
#define STATE_SPACE_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>

namespace muse_mcl_2d {
struct StateSpaceDescription2D
{
    using sample_t               = Sample2D;
    using state_t                = typename cslibs_math_2d::Pose2d<double>;
    using state_space_boundary_t = typename cslibs_math_2d::Point2d<double>;
    using transform_t            = typename cslibs_math_2d::Transform2d<double>;
    using covariance_t           = typename cslibs_math_2d::Covariance2d<double>;
};
}

#endif // STATE_SPACE_2D_HPP
