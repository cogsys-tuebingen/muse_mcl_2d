#ifndef SAMPLE_INDEXATION_2D_HPP
#define SAMPLE_INDEXATION_2D_HPP

#include <cslibs_math/common/index.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>

namespace muse_mcl_2d {
class SampleIndexation2D {
public:
    using resolution_t  = std::array<double, 2>;
    using index_t       = std::array<int, 3>;
    using size_t        = std::array<std::size_t, 2>;
    using state_t       = Sample2D::state_t;

    inline SampleIndexation2D()
    {
        resolution_.fill(0.0);
    }

    inline SampleIndexation2D(const resolution_t &resolution) :
        resolution_(resolution),
        resolution_inv_{{1.0 / resolution[0],
                         1.0 / resolution[1]}}
    {
    }

    inline void setResolution(const resolution_t &resolution)
    {
        resolution_ = resolution;
        resolution_inv_ =
               {{1.0 / resolution[0],
                 1.0 / resolution[1]}};
    }

    inline resolution_t const & getResolution() const
    {
        return resolution_;
    }

    inline index_t create(const Sample2D &sample) const
    {
        return index_t({
                             {static_cast<int>(std::floor(sample.state.tx()            * resolution_inv_[0])),
                              static_cast<int>(std::floor(sample.state.ty()            * resolution_inv_[0])),
                              static_cast<int>(cslibs_math::common::angle::normalize2Pi(sample.state.yaw())  * resolution_inv_[1])},
                         });
    }

    inline index_t create(const state_t &state) const
    {
        return index_t({
                             {static_cast<int>(std::floor(state.tx()            * resolution_inv_[0])),
                              static_cast<int>(std::floor(state.ty()            * resolution_inv_[0])),
                              static_cast<int>(cslibs_math::common::angle::normalize2Pi(state.yaw())  * resolution_inv_[1])},
                         });
    }

    inline index_t create(const std::array<double, 3> &state) const
    {
        return index_t({
                             {static_cast<int>(std::floor(state[0] / resolution_[0])),
                              static_cast<int>(std::floor(state[1] / resolution_[0])),
                              static_cast<int>(cslibs_math::common::angle::normalize2Pi(state[2])  * resolution_inv_[1])}
                         });
    }

private:
    resolution_t resolution_;
    resolution_t resolution_inv_;
};
}

#endif // SAMPLE_INDEXATION_2D_HPP
