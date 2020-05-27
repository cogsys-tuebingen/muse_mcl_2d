#ifndef MUSE_MCL_2D_SAMPLE_2D_HPP
#define MUSE_MCL_2D_SAMPLE_2D_HPP

#include <cslibs_math_2d/linear/covariance.hpp>
#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_time/rate.hpp>
#include <memory>
#include <muse_smc/smc/traits/sample.hpp>

namespace muse_mcl_2d {
struct EIGEN_ALIGN16 Hypothesis2D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using allocator_t = Eigen::aligned_allocator<Hypothesis2D>;
  using Ptr = std::shared_ptr<Hypothesis2D>;
  using state_t = cslibs_math_2d::Pose2d;

  state_t state;

  inline Hypothesis2D() = default;
  inline Hypothesis2D(const Hypothesis2D &other) = default;
  inline Hypothesis2D(Hypothesis2D &&other) = default;
  inline Hypothesis2D &operator=(const Hypothesis2D &other) = default;
  inline Hypothesis2D &operator=(Hypothesis2D &&other) = default;
};
}  // namespace muse_mcl_2d

namespace muse_smc {
namespace traits {
/**
 * @brief Defines the data type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template <>
struct Data<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_plugins_data::Data;
};

/**
 * @brief Defines the data provider type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template <>
struct DataProvider<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_plugins_data::DataProvider;
};

/**
 * @brief Defines the state type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template <>
struct State<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_math_2d::Pose2d;
};

template <>
struct StateAccess<muse_mcl_2d::Hypothesis2D> {
  static inline cslibs_math_2d::Pose2d &get(muse_mcl_2d::Hypothesis2D &hypothesis) {
    return hypothesis.state;
  }
  static inline cslibs_math_2d::Pose2d const &get(
      const muse_mcl_2d::Hypothesis2D &hypothesis) {
    return hypothesis.state;
  }
};

/**
 * @brief Defines the transform type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template <>
struct Transform<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_math_2d::Transform2d;
};

/**
 * @brief Defines the covariance type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template <>
struct Covariance<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_math_2d::Covariance2d;
};

/**
 * @brief Defines the state space bounary type the filter is working with.
 * @tparam SampleType   sample type to set the trait for
 */
template <>
struct StateSpaceBoundary<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_math_2d::Point2d;
};
template <>
struct Time<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_time::Time;
};
template <>
struct TimeFrame<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_time::TimeFrame;
};
template <>
struct Duration<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_time::Duration;
};

template <>
struct Rate<muse_mcl_2d::Hypothesis2D> {
  using type = cslibs_time::Rate;
};
}  // namespace traits
}  // namespace muse_smc

namespace muse_mcl_2d {
using Sample2D = muse_smc::traits::Sample<Hypothesis2D>::type;
}
#endif  // MUSE_MCL_2D_SAMPLE_2D_HPP
