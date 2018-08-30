#ifndef STATE_PUBLISHER_H
#define STATE_PUBLISHER_H

#include <muse_smc/smc/smc_state.hpp>
#include <cslibs_utility/logger/csv_logger.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/tf/tf_publisher.hpp>
#include <muse_mcl_2d/state_space/state_space_description_2d.hpp>

#include "sample_set_publisher_2d.h"

namespace muse_mcl_2d {
class StatePublisher : public muse_smc::SMCState<StateSpaceDescription2D>
{
public:
    using Ptr = std::shared_ptr<StatePublisher>;
    using stamped_t = cslibs_math::utility::Stamped<cslibs_math_2d::Transform2d>;

    StatePublisher();
    virtual ~StatePublisher();

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

    stamped_t                             latest_w_T_b_;
    cslibs_math_2d::Covariance3d          latest_w_T_b_covariance_;

    void publishState(const sample_set_t::ConstPtr &sample_set);
};
}

#endif // STATE_PUBLISHER_H
