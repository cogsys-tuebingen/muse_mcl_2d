#ifndef STATE_PUBLISHER_H
#define STATE_PUBLISHER_H

#include <muse_mcl_2d/samples/sample_2d.hpp>

#include <muse_smc/smc/smc_state.hpp>
#include <cslibs_utility/logger/csv_logger.hpp>

#include <muse_mcl_2d/tf/tf_publisher.hpp>

#include "sample_set_publisher_2d.h"

namespace muse_mcl_2d {
class EIGEN_ALIGN16 StatePublisher : public muse_smc::SMCState<Sample2D>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<StatePublisher>;
    using transform_t  = muse_smc::traits::Transform<Sample2D>::type;
    using covariance_t = muse_smc::traits::Covariance<Sample2D>::type;
    using stamped_t    = cslibs_math::utility::Stamped<transform_t>;

    StatePublisher();

    void setup(ros::NodeHandle &nh);

    virtual void publish(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishIntermediate(const typename sample_set_t::ConstPtr &sample_set) override;
    virtual void publishConstant(const typename sample_set_t::ConstPtr &sample_set) override;

private:
    TFPublisher::Ptr            tf_publisher_;
    SampleSetPublisher2D::Ptr   sample_publisher_;

    std::string                 world_frame_;
    std::string                 odom_frame_;
    std::string                 base_frame_;

    stamped_t                   latest_w_T_b_;
    covariance_t                latest_w_T_b_covariance_;

    void publishState(const sample_set_t::ConstPtr &sample_set);
};
}

#endif // STATE_PUBLISHER_H
