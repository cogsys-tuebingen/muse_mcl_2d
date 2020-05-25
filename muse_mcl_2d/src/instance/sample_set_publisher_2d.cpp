#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <muse_mcl_2d/instance/sample_set_publisher_2d.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

namespace muse_mcl_2d {
namespace color {
#define __HSV2RGB__(H, S, V, R, G, B)     \
  {                                       \
    double _h = H / 60.;                  \
    int _hf = (int)floor(_h);             \
    int _hi = ((int)_h) % 6;              \
    double _f = _h - _hf;                 \
                                          \
    double _p = V * (1. - S);             \
    double _q = V * (1. - _f * S);        \
    double _t = V * (1. - (1. - _f) * S); \
                                          \
    switch (_hi) {                        \
      case 0:                             \
        R = V;                            \
        G = _t;                           \
        B = _p;                           \
        break;                            \
      case 1:                             \
        R = _q;                           \
        G = V;                            \
        B = _p;                           \
        break;                            \
      case 2:                             \
        R = _p;                           \
        G = V;                            \
        B = _t;                           \
        break;                            \
      case 3:                             \
        R = _p;                           \
        G = _q;                           \
        B = V;                            \
        break;                            \
      case 4:                             \
        R = _t;                           \
        G = _p;                           \
        B = V;                            \
        break;                            \
      case 5:                             \
        R = V;                            \
        G = _p;                           \
        B = _q;                           \
        break;                            \
    }                                     \
  }
/// 0 - 120 deg
}  // namespace color

SampleSetPublisher2D::~SampleSetPublisher2D() { end(); }

void SampleSetPublisher2D::setup(ros::NodeHandle &nh) {
  publish_poses_ = nh.param<bool>("publish_poses", true);
  publish_mean_ = nh.param<bool>("publish_means", true);
  publish_markers_ = nh.param<bool>("publish_markers", false);
  publish_samples_ = nh.param<bool>("publish_samples", false);

  const std::string topic_poses =
      nh.param<std::string>("topic_poses", "muse_mcl_2d/poses");
  const std::string topic_mean =
      nh.param<std::string>("topic_mean", "muse_mcl_2d/mean");
  const std::string topic_markers =
      nh.param<std::string>("topic_markers", "muse_mcl_2d/markers");
  const std::string topic_samples =
      nh.param<std::string>("topic_samples", "muse_mcl_2d/samples");

  world_frame_ = nh.param<std::string>("world_frame", "world");

  if (publish_poses_) {
    pub_poses_ = nh.advertise<geometry_msgs::PoseArray>(topic_poses, 1);
  }
  if (publish_mean_) {
    pub_mean_ =
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_mean, 1);
  }
  if (publish_markers_) {
    pub_markers_ =
        nh.advertise<visualization_msgs::MarkerArray>(topic_markers, 1);
  }
  if (publish_samples_) {
    publish_samples_ =
        nh.advertise<muse_mcl_2d::SampleSetMsg>(topic_samples, 1);
  }
}

bool SampleSetPublisher2D::start() {
  if (!worker_thread_.joinable()) {
    stop_ = false;
    worker_thread_ = std::thread([this]() { loop(); });
    return true;
  }
  return false;
}

bool SampleSetPublisher2D::end() {
  if (worker_thread_.joinable()) {
    stop_ = true;
    notify_.notify_one();
    worker_thread_.join();
    return true;
  }
  return false;
}

void SampleSetPublisher2D::set(const sample_vector_t &sample_vector,
                               const double maximum_weight, const state_t &mean,
                               const covariance_t &covariance,
                               const time_t &stamp) {
  lock_t lock(data_mutex_);
  stamp_ = stamp;
  if (publish_mean_) {
    mean_ = mean;
    covariance_ = covariance;
    notify_.notify_one();
  }
  if (publish_markers_ || publish_poses_ || publish_samples_) {
    sample_.reset(new sample_vector_t(sample_vector));
    maximum_weight_ = maximum_weight;
    notify_.notify_one();
  }
}

void SampleSetPublisher2D::loop() {
  auto create_empty_marker = [this]() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.ns = "muse_mcl_2d";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.scale.x = 0.25;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    return marker;
  };

  auto create_pose = [](const state_t &pose) {
    geometry_msgs::Pose p;
    p.position.x = pose.tx();
    p.position.y = pose.ty();
    p.orientation = tf::createQuaternionMsgFromYaw(pose.yaw());
    return p;
  };

  auto create_marker = [&create_empty_marker, &create_pose](
                           const Sample2D &sample,
                           const double maximum_weight) {
    visualization_msgs::Marker m = create_empty_marker();
    m.pose = create_pose(sample.state);
    m.color.a = 1.f;
    if (maximum_weight > 0.0) {
      __HSV2RGB__(120.0 * sample.weight / maximum_weight, 1.0, 1.0, m.color.r,
                  m.color.g, m.color.b);
    } else {
      __HSV2RGB__(0.0, 1.0, 1.0, m.color.r, m.color.g, m.color.b);
    }
    return m;
  };

  auto create_sample = [&create_pose](const Sample2D &sample) {
    SampleMsg msg;
    msg.pose = create_pose(sample.state);
    msg.weight.data = sample.weight;
    return msg;
  };

  lock_t notify_lock(notify_mutex_);

  time_t stamp;
  sample_vector_t::Ptr samples;
  state_t mean;
  covariance_t covariance;
  double maximum_weight = 0.0;

  while (!stop_) {
    notify_.wait(notify_lock);

    {
      lock_t lock(data_mutex_);
      stamp = stamp_;
      if (publish_mean_) {
        mean = mean_;
        covariance = covariance_;
      }
      if (publish_poses_ || publish_markers_ || publish_samples_) {
        samples = std::move(sample_);
        maximum_weight = maximum_weight_;
      }
    }

    std_msgs::Header header;
    header.stamp.fromNSec(stamp.nanoseconds());
    header.frame_id = world_frame_;

    SampleSetMsg::Ptr sample_set_msg;
    visualization_msgs::MarkerArray::Ptr marker_array_msg;
    geometry_msgs::PoseArray::Ptr pose_array_msg;
    geometry_msgs::PoseWithCovarianceStamped::Ptr mean_msg;

    if (publish_samples_) {
      sample_set_msg.reset(new SampleSetMsg);
      sample_set_msg->header = header;
    }
    if (publish_markers_) {
      marker_array_msg.reset(new visualization_msgs::MarkerArray);

#if ROS_VERSION_MAJOR >= 1 && ROS_VERSION_MINOR > 11
      {
        visualization_msgs::Marker m = create_empty_marker();
        m.id = -1;
        m.action = visualization_msgs::Marker::DELETEALL;
        marker_array_msg->markers.emplace_back(m);
      }
#else
      {
        visualization_msgs::Marker m = create_empty_marker();
        m.action = visualization_msgs::Marker::DELETE;
        for (std::size_t i = 0; i < marker_count_; ++i) {
          m.id = i;
          marker_array_msg->markers.emplace_back(m);
        }
      }
#endif
    }
    if (publish_poses_) {
      pose_array_msg.reset(new geometry_msgs::PoseArray);
      pose_array_msg->header = header;
    }

    if (publish_markers_ || publish_samples_ || publish_poses_) {
      std::size_t id = 0;
      for (const Sample2D &sample : *samples) {
        if (publish_markers_) {
          auto m = create_marker(sample, maximum_weight);
          m.id = ++id;
          m.header = header;
          marker_array_msg->markers.emplace_back(m);
        }
        if (publish_poses_) {
          auto p = create_pose(sample.state);
          pose_array_msg->poses.emplace_back(p);
        }
        if (publish_samples_) {
          auto s = create_sample(sample);
          sample_set_msg->samples.emplace_back(s);
        }
      }
    }

    if (publish_markers_) {
      marker_count_ = samples->size();
      pub_markers_.publish(marker_array_msg);
    }
    if (publish_poses_) {
      pub_poses_.publish(pose_array_msg);
    }
    if (publish_samples_) {
      pub_samples_.publish(sample_set_msg);
    }
    if (publish_mean_) {
      mean_msg.reset(new geometry_msgs::PoseWithCovarianceStamped);
      mean_msg->header = header;
      mean_msg->pose.pose = create_pose(mean);
      mean_msg->pose.covariance[0 * 6 + 0] = covariance(0, 0);
      mean_msg->pose.covariance[0 * 6 + 1] = covariance(0, 1);
      mean_msg->pose.covariance[1 * 6 + 0] = covariance(1, 0);
      mean_msg->pose.covariance[1 * 6 + 1] = covariance(1, 1);
      mean_msg->pose.covariance[5 * 6 + 5] = covariance(2, 2);
      pub_mean_.publish(mean_msg);
    }
  }
}
}  // namespace muse_mcl_2d
