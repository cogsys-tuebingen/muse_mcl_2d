#ifndef MUSE_MCL_2D_STATE_PUBLISHER_H
#define MUSE_MCL_2D_STATE_PUBLISHER_H

#include <muse_mcl_2d/instance/sample_set_publisher_2d.h>

#include <cslibs_utility/logger/csv_logger.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_mcl_2d/tf/tf_publisher.hpp>
#include <muse_smc/smc/traits/state_publisher.hpp>

namespace muse_mcl_2d {
class EIGEN_ALIGN16 StatePublisher
    : public muse_smc::traits::StatePublisher<Sample2D>::type {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<StatePublisher>;
  using transform_t = muse_smc::traits::Transform<Sample2D>::type;
  using covariance_t = muse_smc::traits::Covariance<Sample2D>::type;
  using stamped_t = cslibs_math::utility::Stamped<transform_t>;
  using sample_set_t = muse_smc::traits::SampleSet<Sample2D>::type;

  StatePublisher();

  void setup(ros::NodeHandle &nh);

  void publish(const sample_set_t::ConstPtr &sample_set) override;
  void publishIntermediate(
      const sample_set_t::ConstPtr &sample_set) override;
  void publishConstant(
      const sample_set_t::ConstPtr &sample_set) override;

 private:
  TFPublisher::Ptr tf_publisher_;
  SampleSetPublisher2D::Ptr sample_publisher_;

  std::string world_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  stamped_t latest_w_T_b_;
  covariance_t latest_w_T_b_covariance_;

  void publishState(const sample_set_t::ConstPtr &sample_set);
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_2D_STATE_PUBLISHER_H
