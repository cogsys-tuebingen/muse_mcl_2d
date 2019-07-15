#ifndef SAMPLE_2D_HPP
#define SAMPLE_2D_HPP

#include <memory>

#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_math_2d/linear/covariance.hpp>
#include <cslibs_plugins_data/data_provider.hpp>

#include <muse_smc/smc/smc_traits.hpp>

namespace muse_mcl_2d {
struct EIGEN_ALIGN16 Sample2D {
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

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<>
struct Data<muse_mcl_2d::Sample2D> {
    using type = cslibs_plugins_data::Data;
};


/**
 * @brief Defines the data provider type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<>
struct DataProvider<muse_mcl_2d::Sample2D> {
    using type = cslibs_plugins_data::DataProvider;
};

/**
 * @brief Defines the state type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<>
struct State<muse_mcl_2d::Sample2D> {
    using type = cslibs_math_2d::Pose2d;
};

/**
 * @brief Defines the transform type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<>
struct Transform<muse_mcl_2d::Sample2D> {
    using type = cslibs_math_2d::Transform2d;
};

/**
 * @brief Defines the covariance type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<>
struct Covariance<muse_mcl_2d::Sample2D> {
    using type = cslibs_math_2d::Covariance2d;
};

/**
 * @brief Defines the state space bounary type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template<>
struct StateSpaceBoundary<muse_mcl_2d::Sample2D> {
    using type = cslibs_math_2d::Point2d;
};
}
}
#endif // SAMPLE_2D_HPP
