#include "state_publisher_2d.h"

#include <muse_mcl_2d/density/sample_density_2d.hpp>

namespace muse_mcl_2d {
StatePublisher::StatePublisher() :
    latest_w_T_b_(cslibs_math_2d::Transform2d(),
                  cslibs_time::Time(ros::Time::now().toNSec()))
{
}

StatePublisher::~StatePublisher()
{
}

void StatePublisher::setup(ros::NodeHandle &nh)
{
    const double pub_rate_tf    = nh.param<double>("pub_rate_tf", 30.0);
    const double tf_timeout     = nh.param<double>("tf_timeout", 0.05);
    const double tf_keep_alive  = nh.param<double>("tf_keep_alive", 0.0);

    world_frame_ = nh.param<std::string>("world_frame", "/world");
    odom_frame_  = nh.param<std::string>("odom_frame", "/odom");
    base_frame_  = nh.param<std::string>("base_frame", "/base_link");

    sample_publisher_.reset(new SampleSetPublisher2D);
    sample_publisher_->setup(nh);
    sample_publisher_->start();

    tf_publisher_.reset(new TFPublisher(pub_rate_tf, odom_frame_, base_frame_, world_frame_, tf_timeout, tf_keep_alive));
    tf_publisher_->start();
}

void StatePublisher::publish(const sample_set_t::ConstPtr &sample_set)
{
    /// calculate the latest transformation
    SampleDensity2D::ConstPtr density =
            std::dynamic_pointer_cast<SampleDensity2D const>(sample_set->getDensity());

    if(!density) {
        std::cerr << "[StatePublisher]: Incomaptible sample density estimation!" << "\n";
        return;
    }

    if(density->maxClusterMean(latest_w_T_b_.data(), latest_w_T_b_covariance_)) {
        latest_w_T_b_.stamp() = sample_set->getStamp();
        /// make sure that TF gets published first #most important
        tf_publisher_->setTransform(latest_w_T_b_);
    }
    /// publish the particle set state
    publishState(sample_set);
}

void StatePublisher::publishIntermediate(const sample_set_t::ConstPtr &sample_set)
{
    publishState(sample_set);
}

void StatePublisher::publishState(const sample_set_t::ConstPtr &sample_set)
{
    sample_publisher_->set(sample_set->getSamples(),
                           sample_set->getMaximumWeight(),
                           latest_w_T_b_,
                           latest_w_T_b_covariance_,
                           sample_set->getStamp());
}
}
