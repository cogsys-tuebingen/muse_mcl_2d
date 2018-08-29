#ifndef TRANSFORM_PUBLISHER_ANCHORED_HPP
#define TRANSFORM_PUBLISHER_ANCHORED_HPP

#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>

#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math_ros/tf/tf_listener_2d.hpp>
#include <cslibs_math_ros/tf/conversion_2d.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cslibs_math_ros/tf/tf_listener_2d.hpp>
#include <cslibs_time/rate.hpp>

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
class TFPublisher {
public:
    using Ptr = std::shared_ptr<TFPublisher>;
    using stamped_t = cslibs_time::Stamped<cslibs_math_2d::Transform2d>;
    using time_t = cslibs_time::Time;

    /**
     * @brief TransformPublisherAnchored constructor.
     * @param rate          - the publication rate
     * @param odom_frame    - the odometry frame
     * @param base_frame    - the base frame
     * @param world_frame   - the world fram
     */
    inline TFPublisher(const double rate,
                       const std::string &odom_frame,
                       const std::string &base_frame,
                       const std::string &world_frame,
                       const double timeout = 0.1,
                       const double tf_keep_alive_for = 0.0) :
        odom_frame_(odom_frame),
        base_frame_(base_frame),
        world_frame_(world_frame),
        timeout_(timeout),
        running_(false),
        stop_(false),
        w_T_o_( tf::Transform(tf::createIdentityQuaternion(),
                              tf::Vector3(0,0,0)),
                ros::Time::now(),
                world_frame_, odom_frame_),
        w_T_b_(cslibs_math_2d::Transform2d::identity(), cslibs_time::Time(ros::Time::now().toNSec())),
        tf_dirty_(false),
        tf_rate_(rate),
        tf_keep_alive_for_(tf_keep_alive_for)
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
        std::unique_lock<std::mutex> l(tf_mutex_);
        if(w_t_b.stamp() > w_T_b_.stamp()) {
            w_T_b_ = w_t_b;
            tf_dirty_ = true;
        }
    }
    inline void renewTimeStamp(const time_t &stamp)
    {
        std::unique_lock<std::mutex> l(tf_mutex_);
        if(stamp > w_T_b_.stamp()) {
            w_T_b_.stamp() = stamp;
            tf_renew_time_ = true;
        }
    }

    inline void resetTransform()
    {
        std::unique_lock<std::mutex> l(tf_mutex_);
        tf_last_update_time_ = cslibs_time::Time(0ul);
    }


private:
    const std::string        odom_frame_;
    const std::string        base_frame_;
    const std::string        world_frame_;
    const ros::Duration      timeout_;

    std::atomic_bool         running_;
    std::atomic_bool         stop_;
    std::thread              worker_thread_;
    std::mutex               tf_mutex_;

    tf::TransformBroadcaster tf_broadcaster_;
    cslibs_math_ros::tf::TFListener2d tf_listener_;

    tf::StampedTransform     w_T_o_;
    ros::Time                time_w_T_o_;
    stamped_t                w_T_b_;
    std::atomic_bool         tf_dirty_;
    std::atomic_bool         tf_renew_time_;
    cslibs_time::Rate        tf_rate_;
    cslibs_time::Time        tf_last_update_time_;
    cslibs_time::Duration    tf_keep_alive_for_;
    cslibs_time::Time        tf_keep_alive_until_;

    inline void loop()
    {
        auto get_updated_tf = [this] () {
            std::unique_lock<std::mutex> l(tf_mutex_);
            stamped_t w_t_b = w_T_b_;
            l.unlock();

            cslibs_math_2d::Transform2d b_T_o = cslibs_math_2d::Transform2d::identity();
            const ros::Time time = ros::Time(w_t_b.stamp().seconds());
            if(tf_listener_.lookupTransform(base_frame_, odom_frame_, time, b_T_o, timeout_)) {
                cslibs_math_2d::Transform2d w_T_o = w_t_b.data() * b_T_o;
                w_T_o_ = tf::StampedTransform( cslibs_math_ros::tf::conversion_2d::from(w_T_o), time, world_frame_, odom_frame_);
                time_w_T_o_ = time;
                return true;
            }
            return false;
        };

        auto renew_time_tf = [this] () {
            std::unique_lock<std::mutex> l(tf_mutex_);
            time_w_T_o_ = ros::Time(w_T_b_.stamp().seconds());
        };

#pragma message "Update to event-based publishing using a condition variable"

        running_ = true;
        while(!stop_) {
            const cslibs_time::Time now = cslibs_time::Time::now();
            if(tf_dirty_) {
                if(get_updated_tf()) {
                    tf_last_update_time_ = now;
                    tf_keep_alive_until_ = now + tf_keep_alive_for_;
                    tf_dirty_     = false;
                    tf_renew_time_ = false;
                }
            } else if(tf_renew_time_) {
                renew_time_tf();
                tf_last_update_time_ = now;
                tf_keep_alive_until_ = now + tf_keep_alive_for_;
                tf_renew_time_ = false;
            }

            if(!tf_last_update_time_.isZero() && now <= tf_keep_alive_until_) {
                /// get time diff here
                const ros::Duration dt((now - tf_last_update_time_).seconds());
                w_T_o_.stamp_ = time_w_T_o_ + dt;
                tf_broadcaster_.sendTransform(w_T_o_);
            }
            tf_rate_.sleep();
        }
        running_ = false;
    }
};
}

#endif // TRANSFORM_PUBLISHER_ANCHORED_HPP
