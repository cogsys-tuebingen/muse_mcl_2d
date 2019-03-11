#ifndef SAMPLE_SET_PUBLISHER_2D_H
#define SAMPLE_SET_PUBLISHER_2D_H

#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <muse_smc/samples/sample_set.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/SampleSetMsg.h>
#include <muse_mcl_2d/state_space/state_space_description_2d.hpp>

namespace muse_mcl_2d {
class SampleSetPublisher2D
{
public:
    using Ptr = std::shared_ptr<SampleSetPublisher2D>;
    using sample_set_t    = muse_smc::SampleSet<StateSpaceDescription2D>;
    using sample_vector_t = sample_set_t::sample_vector_t;
    using time_t          = cslibs_time::Time;
    using lock_t          = std::unique_lock<std::mutex>;

    using state_t         = StateSpaceDescription2D::state_t;
    using covariance_t    = StateSpaceDescription2D::covariance_t;

    SampleSetPublisher2D();
    virtual ~SampleSetPublisher2D();

    void setup(ros::NodeHandle &nh);
    bool start();
    bool end();

    void set(const sample_vector_t  &sample_vector,
             const double            maximum_weight,
             const state_t          &mean,
             const covariance_t     &covariance,
             const time_t           &stamp);

private:
    std::atomic_bool                    running_;
    std::atomic_bool                    stop_;
    std::thread                         worker_thread_;
    std::condition_variable             notify_;
    std::mutex                          notify_mutex_;

    std::mutex                          data_mutex_;
    sample_vector_t::Ptr                sample_;
    double                              maximum_weight_;
    state_t                             mean_;
    covariance_t                        covariance_;
    time_t                              stamp_;

    ros::Publisher                      pub_markers_;
    ros::Publisher                      pub_samples_;
    ros::Publisher                      pub_poses_;
    ros::Publisher                      pub_mean_;
    std::size_t                         marker_count_;

    std::string                         world_frame_;

    bool publish_poses_;
    bool publish_markers_;
    bool publish_samples_;
    bool publish_mean_;

    void loop();
};
}

#endif // SAMPLE_SET_PUBLISHER_2D_H
