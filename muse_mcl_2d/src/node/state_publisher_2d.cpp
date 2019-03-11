#include <muse_mcl_2d/node/state_publisher_2d.h>

#include <muse_mcl_2d/density/sample_density_2d.hpp>

namespace muse_mcl_2d {
StatePublisher::StatePublisher() :
    latest_w_T_b_(transform_t(),
                  cslibs_time::Time(ros::Time::now().toNSec()).time())
{
}

StatePublisher::~StatePublisher()
{
}

void StatePublisher::setup(ros::NodeHandle &nh)
{
    const double tf_timeout     = nh.param<double>("tf_timeout", 0.05);
    const double tf_keep_alive  = nh.param<double>("tf_tolerance", 0.0);
    const bool   tf_publish     = nh.param<bool>("tf_publish", true);

    world_frame_ = nh.param<std::string>("world_frame", "/world");
    odom_frame_  = nh.param<std::string>("odom_frame", "/odom");
    base_frame_  = nh.param<std::string>("base_frame", "/base_link");

    sample_publisher_.reset(new SampleSetPublisher2D);
    sample_publisher_->setup(nh);
    sample_publisher_->start();

    if (tf_publish)
    {
        tf_publisher_.reset(new TFPublisher(odom_frame_, base_frame_, world_frame_, tf_timeout, tf_keep_alive));
        tf_publisher_->start();
    }
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
        latest_w_T_b_.stamp() = sample_set->getStamp().time();

        /// make sure that TF gets published first #most important
        if (tf_publisher_) {
            tf_publisher_->renewTimeStamp(sample_set->getStamp());
            tf_publisher_->setTransform(latest_w_T_b_);
        }
    }

    /// publish the particle set state
    publishState(sample_set);
}

void StatePublisher::publishIntermediate(const sample_set_t::ConstPtr &sample_set)
{
    if (tf_publisher_)
        tf_publisher_->renewTimeStamp(sample_set->getStamp());

    publishState(sample_set);
}

void StatePublisher::publishConstant(const sample_set_t::ConstPtr &sample_set)
{
    if (tf_publisher_)
        tf_publisher_->renewTimeStamp(sample_set->getStamp());

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
