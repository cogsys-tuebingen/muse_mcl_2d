#ifndef TRANSFORM_PUBLISHER_ANCHORED_HPP
#define TRANSFORM_PUBLISHER_ANCHORED_HPP

#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>

#include <cslibs_math/utility/stamped.hpp>
#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math_ros/tf/tf_listener.hpp>
#include <cslibs_math_ros/tf/conversion_2d.hpp>

#include <muse_smc/smc/traits.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cslibs_time/rate.hpp>
#include <cslibs_utility/synchronized/synchronized_queue.hpp>


#include <ros/time.h>

namespace muse_mcl_2d {
/**
 * @brief The TransformPublisherAnchored class can be used to published
 *        transformations consisting of a relative transformation in a moving
 *        coordinate frame and a anchor transformation in a fixed frame.
 *        More precisely, relative motion of the base link in the odometry frame
 *        is calculated and combined with the fixed anchor transformation in the
 *        world frame.
 *
 */
class EIGEN_ALIGN16 TFPublisher {
public:
    using Ptr = std::shared_ptr<TFPublisher>;
    using transform_t = muse_smc::traits::Transform<Sample2D>::type;
    using stamped_t   = cslibs_math::utility::Stamped<transform_t>;
    using time_t  = cslibs_time::Time;
    using queue_t = cslibs_utility::synchronized::queue<stamped_t, std::deque<stamped_t, stamped_t::allocator_t>>;
    using mutex_t = std::mutex;
    using lock_t  = std::unique_lock<mutex_t>;
    using condition_variable_t  = std::condition_variable;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief TransformPublisherAnchored constructor.
     * @param rate          - the publication rate
     * @param odom_frame    - the odometry frame
     * @param base_frame    - the base frame
     * @param world_frame   - the world fram
     */
    inline TFPublisher(const std::string &odom_frame,
                       const std::string &base_frame,
                       const std::string &world_frame,
                       const double timeout = 0.1,
                       const double tolerance = 0.0) :
        odom_frame_(odom_frame),
        base_frame_(base_frame),
        world_frame_(world_frame),
        timeout_(timeout),
        running_(false),
        stop_(false),
        w_T_o_(tf::Transform(tf::createIdentityQuaternion(),
                             tf::Vector3(0,0,0)),
               ros::Time(0),
               world_frame_, odom_frame_),
        w_T_b_(transform_t::identity(), cslibs_time::Time(0ul).time()),
        tf_tolerance_(tolerance),
        tf_last_update_(0)
    {
    }

    virtual ~TFPublisher()
    {
        end();
    }

    inline void start()
    {
        if(running_)
            return;

        worker_thread_ = std::thread([this]{loop();});
    }

    inline void end()
    {
        if(!running_)
            return;

        stop_ = true;
        if(worker_thread_.joinable())
            worker_thread_.join();
    }

    inline void setTransform(const stamped_t &w_t_b)
    {
        poses_.push(w_t_b);
        notify_event_.notify_one();
    }

    inline void renewTimeStamp(const time_t &stamp)
    {
        lock_t l(tf_renew_time_stamp_mutex_);
        tf_renew_time_stamp_ = stamp;
        tf_renew_time_ = true;
        notify_event_.notify_one();
    }

private:
    const std::string                   odom_frame_;
    const std::string                   base_frame_;
    const std::string                   world_frame_;
    const ros::Duration                 timeout_;

    std::atomic_bool                    running_;
    std::atomic_bool                    stop_;
    std::thread                         worker_thread_;
    condition_variable_t                notify_event_;
    mutex_t                             notify_event_mutex_;

    tf::TransformBroadcaster            tf_broadcaster_;
    cslibs_math_ros::tf::TFListener     tf_listener_;

    tf::StampedTransform                w_T_o_;
    stamped_t                           w_T_b_;

    queue_t                             poses_;

    std::atomic_bool                    tf_renew_time_;
    mutex_t                             tf_renew_time_stamp_mutex_;
    cslibs_time::Time                   tf_renew_time_stamp_;
    cslibs_time::Time                   tf_time_w_T_o_;
    ros::Duration                       tf_tolerance_;
    ros::Time                           tf_last_update_;

    inline void loop()
    {
        auto update_tf = [this] (const stamped_t &w_t_b) {
            transform_t b_T_o = transform_t::identity();
            const ros::Time time = ros::Time(cslibs_math::utility::tiny_time::seconds(w_t_b.stamp()));
            if(tf_listener_.lookupTransform(base_frame_, odom_frame_, time, b_T_o, timeout_)) {
                transform_t w_T_o = w_t_b.data() * b_T_o;
                w_T_o_ = tf::StampedTransform( cslibs_math_ros::tf::conversion_2d::from(w_T_o), time, world_frame_, odom_frame_);
                tf_time_w_T_o_ = cslibs_time::Time(w_t_b.stamp());
                return true;
            }
            return false;
        };

        auto renew_time_tf = [this] () {
            tf_time_w_T_o_ = tf_renew_time_stamp_;
            tf_renew_time_ = false;
        };

        running_ = true;
        lock_t notify_event_mutex_lock(notify_event_mutex_);
        while(!stop_) {
            notify_event_.wait(notify_event_mutex_lock);
            ros::Time now = ros::Time::now();

            while(!poses_.empty()) {
                const stamped_t w_T_b = poses_.pop();
                if(w_T_b.stamp() >= tf_time_w_T_o_.time()) {
                    update_tf(w_T_b);
                    w_T_o_.stamp_ = ros::Time(tf_time_w_T_o_.seconds()) + tf_tolerance_;
                    tf_broadcaster_.sendTransform(w_T_o_);
                    tf_last_update_ = now;
                }
            }
            if(tf_renew_time_) {
                if(tf_renew_time_stamp_ >= tf_time_w_T_o_) {
                    renew_time_tf();
                    w_T_o_.stamp_ = ros::Time(tf_time_w_T_o_.seconds()) + tf_tolerance_;
                    tf_broadcaster_.sendTransform(w_T_o_);
                    tf_last_update_ = now;
                }
            }

        }
        running_ = false;
    }
};
}

#endif // TRANSFORM_PUBLISHER_ANCHORED_HPP
