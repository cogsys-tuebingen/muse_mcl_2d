#ifndef MUSE_MCL_NODE_H
#define MUSE_MCL_NODE_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <muse_mcl_2d/GlobalInitialization.h>
#include <muse_mcl_2d/PoseInitialization.h>
#include <muse_mcl_2d/instance/state_publisher_2d.h>
#include <ros/ros.h>
#include <tf/time_cache.h>

#include <cslibs_math_ros/tf/tf_listener.hpp>
#include <cslibs_plugins/plugin_factory.hpp>
#include <cslibs_plugins/plugin_loader.hpp>
#include <muse_mcl_2d/density/sample_density_2d.hpp>
#include <muse_mcl_2d/map/map_provider_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_integral_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_integral_amcl_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_model_2d.hpp>
#include <muse_mcl_2d/resampling/resampling_2d.hpp>
#include <muse_mcl_2d/instance/sample_2d.hpp>
#include <muse_mcl_2d/sampling/normal_sampling_2d.hpp>
#include <muse_mcl_2d/sampling/uniform_sampling_2d.hpp>
#include <muse_mcl_2d/scheduling/scheduler_2d.hpp>
#include <muse_mcl_2d/update/update_model_2d.hpp>
#include <muse_smc/smc/smc.hpp>
#include <muse_smc/smc/traits/update_relay.hpp>
#include <muse_smc/smc/traits/prediction_relay.hpp>

namespace muse_mcl_2d {
class MuseMCL2DNode {
 public:
  MuseMCL2DNode();
  virtual ~MuseMCL2DNode();

  bool setup();

  void start();

  bool requestGlobalInitialization(
      muse_mcl_2d::GlobalInitialization::Request &req,
      muse_mcl_2d::GlobalInitialization::Response &res);

  bool requestPoseInitialization(
      muse_mcl_2d::PoseInitialization::Request &req,
      muse_mcl_2d::PoseInitialization::Response &res);

  void poseInitialization(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

 private:
  using data_t = cslibs_plugins_data::Data;
  using data_provider_t = cslibs_plugins_data::DataProvider;
  using covariance_t = muse_smc::traits::Covariance<Sample2D>::type;

  using map_provider_map_t = std::map<std::string, MapProvider2D::Ptr>;
  using data_provider_map_t =
      std::map<std::string, typename data_provider_t::Ptr>;
  using update_model_map_t = std::map<std::string, UpdateModel2D::Ptr>;

  using smc_t = muse_smc::SMC<Sample2D>;
  using UpdateRelay2D = muse_smc::traits::UpdateRelay<Sample2D>::type;
  using PredictionRelay2D = muse_smc::traits::PredictionRelay<Sample2D>::type;
  using sample_set_t = muse_smc::traits::SampleSet<Sample2D>::type;
  using prediction_integrals_t =
      muse_smc::traits::PredictionIntegrals<Sample2D>::type;

  using update_model_mapping_t = UpdateRelay2D::map_t;

  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_public_;
  ros::ServiceServer initialization_service_global_;
  ros::ServiceServer initialization_service_pose_;
  ros::Subscriber initialization_subscriber_pose_;

  //// data providers
  cslibs_math_ros::tf::TFListener::Ptr
      tf_provider_frontend_;  /// for data providers and data conversion
  cslibs_math_ros::tf::TFListener::Ptr
      tf_provider_backend_;  /// for the backend (the particle filter and the
                             /// sensor updates)
  map_provider_map_t map_providers_;
  data_provider_map_t data_providers_;

  smc_t::Ptr particle_filter_;
  prediction_integrals_t::Ptr prediction_integrals_;
  sample_set_t::Ptr sample_set_;

  SampleDensity2D::Ptr sample_density_;
  StatePublisher::Ptr state_publisher_;

  //// prediction & update
  update_model_map_t update_models_;
  PredictionModel2D::Ptr prediction_model_;

  /// sampling & resampling
  UniformSampling2D::Ptr uniform_sampling_;
  NormalSampling2D::Ptr normal_sampling_;
  Resampling2D::Ptr resampling_;
  Scheduler2D::Ptr scheduler_;

  UpdateRelay2D::Ptr update_forwarder_;
  PredictionRelay2D::Ptr prediction_forwarder_;

  void checkPoseInitialization();
  bool getUpdateModelProviderMapping(update_model_mapping_t &update_mapping);
  bool getPredictionProvider(
      cslibs_plugins_data::DataProvider::Ptr &prediction_provider);
};
}  // namespace muse_mcl_2d

#endif  // MUSE_MCL_NODE_H
