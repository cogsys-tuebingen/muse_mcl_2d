#include "muse_mcl_2d_node.h"

#include <cslibs_math_ros/tf/conversion_2d.hpp>
#include <cslibs_math/common/angle.hpp>

namespace muse_mcl_2d{
MuseMCL2DNode::MuseMCL2DNode() :
    nh_private_("~"),
    tf_provider_frontend_(new cslibs_math_ros::tf::TFListener),
    tf_provider_backend_(new cslibs_math_ros::tf::TFListener)
{
}

MuseMCL2DNode::~MuseMCL2DNode()
{
    for(auto &d : data_providers_) {
        d.second->disable();
    }
}

void MuseMCL2DNode::start()
{
    for(auto &d : data_providers_) {
        d.second->enable();
    }

    if(!particle_filter_->start()) {
        ROS_ERROR_STREAM("Couldn't start the filter!");
        return;
    }

    /// check if there is an initial pose set
    checkPoseInitialization();

    double node_rate = nh_private_.param<double>("node_rate", 60.0);
    if(node_rate == 0.0) {
        /// unlimited speed
        ROS_INFO_STREAM("Spinning without rate!");
        ros::spin();
    } else {
        /// limited speed
        ros::WallRate r(node_rate);
        ROS_INFO_STREAM("Spinning with " << node_rate << " Hz!");
        while(ros::ok()) {
            ros::spinOnce();
            r.sleep();
        }
    }
}

bool MuseMCL2DNode::requestGlobalInitialization(muse_mcl_2d::GlobalInitialization::Request &req,
                                                muse_mcl_2d::GlobalInitialization::Response &res)
{
    auto get_time = []() {
        return cslibs_time::Time(ros::Time::now().toNSec());
    };

    particle_filter_->requestUniformInitialization(get_time());
    res.success = true;
    return true;
}

bool MuseMCL2DNode::requestPoseInitialization(muse_mcl_2d::PoseInitialization::Request &req,
                                              muse_mcl_2d::PoseInitialization::Response &res)
{
    auto convert_pose = [&req]() {
        tf::Pose p;
        tf::poseMsgToTF(req.pose.pose, p);
        return  cslibs_math_ros::tf::conversion_2d::from(p);
    };
    auto convert_covariance = [&req]() {
        cslibs_math_2d::Covariance3d cov;
        for(std::size_t i = 0 ; i < 2 ; ++i) {
            for(std::size_t j = 0 ; j < 2 ; ++j) {
                cov(i,j) = req.pose.covariance[6*i+j];
            }
        }
        cov(2,2) = req.pose.covariance[6*5+5];
        return cov;
    };

    auto get_time = []() {
        return cslibs_time::Time(ros::Time::now().toNSec());
    };

    particle_filter_->requestStateInitialization(convert_pose(), convert_covariance(), get_time());
    res.success = true;
    return true;
}

void MuseMCL2DNode::poseInitialization(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    auto convert_pose = [&msg]() {
        tf::Pose p;
        tf::poseMsgToTF(msg->pose.pose, p);
        return  cslibs_math_ros::tf::conversion_2d::from(p);
    };
    auto convert_covariance = [&msg]() {
        cslibs_math_2d::Covariance3d cov;
        for(std::size_t i = 0 ; i < 2 ; ++i) {
            for(std::size_t j = 0 ; j < 2 ; ++j) {
                cov(i,j) = msg->pose.covariance[6*i+j];
            }
        }
        cov(2,2) = msg->pose.covariance[6*5+5];
        return cov;
    };
    auto convert_time = [&msg](){
        return cslibs_time::Time(msg->header.stamp.toNSec());
    };

    particle_filter_->requestStateInitialization(convert_pose(), convert_covariance(), convert_time());
}

bool MuseMCL2DNode::setup()
{
    /// load all plugins
    cslibs_plugins::PluginLoader loader("muse_mcl_2d", nh_private_);

    {   /// Update Models
        loader.load<UpdateModel2D, cslibs_math_ros::tf::TFProvider::Ptr, ros::NodeHandle&>(update_models_, tf_provider_backend_, nh_private_);
        if(update_models_.empty()) {
            ROS_ERROR_STREAM("No update model functions were found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        std::string update_model_list = "[";
        for(auto &e : update_models_) {
            update_model_list += e.first + ",";
        }
        update_model_list.at(update_model_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded update function models.");
        ROS_INFO_STREAM(update_model_list);
    }
    {   /// Prediction Model
        loader.load<PredictionModel2D, cslibs_math_ros::tf::TFProvider::Ptr, ros::NodeHandle&>(prediction_model_, tf_provider_backend_, nh_private_);
        if(!prediction_model_) {
            ROS_ERROR_STREAM("No prediction model functions was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded prediction function model.");
        ROS_INFO_STREAM("[" << prediction_model_->getName() << "]");
    }
    {   /// Map Providers
        loader.load<MapProvider2D, ros::NodeHandle&>(map_providers_, nh_private_);
        if(map_providers_.empty()) {
            ROS_ERROR_STREAM("No map provider was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        std::string map_provider_list = "[";
        for(auto &e : map_providers_) {
            map_provider_list += e.first + ",";
        }
        map_provider_list.at(map_provider_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded map providers.");
        ROS_INFO_STREAM(map_provider_list);
    }
    {   /// Data Providers
        /// load data plugins
        cslibs_plugins::PluginLoader data_loader("cslibs_plugins_data", nh_private_);
        data_loader.load<cslibs_plugins_data::DataProvider, cslibs_math_ros::tf::TFProvider::Ptr, ros::NodeHandle&>(data_providers_, tf_provider_frontend_, nh_private_);
        if(data_providers_.empty()) {
            ROS_ERROR_STREAM("No data provider was found!");
            return false;
        }
        std::string data_provider_list = "[";
        for(auto &e : data_providers_) {
            data_provider_list += e.first + ",";
        }
        data_provider_list.at(data_provider_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded data providers.");
        ROS_INFO_STREAM(data_provider_list);
    }
    { /// sampling algorithms
        loader.load<UniformSampling2D, map_provider_map_t, cslibs_math_ros::tf::TFProvider::Ptr, ros::NodeHandle&>(uniform_sampling_, map_providers_, tf_provider_backend_, nh_private_);
        if(!uniform_sampling_) {
            ROS_ERROR_STREAM("No uniform sampling function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded uniform sampler.");
        ROS_INFO_STREAM("[" << uniform_sampling_->getName() << "]");
        loader.load<NormalSampling2D, map_provider_map_t, cslibs_math_ros::tf::TFProvider::Ptr, ros::NodeHandle&>(normal_sampling_, map_providers_,  tf_provider_backend_, nh_private_);
        if(!normal_sampling_) {
            ROS_ERROR_STREAM("No gaussian sampling function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded gaussian sampler.");
        ROS_INFO_STREAM("[" << normal_sampling_->getName() << "]");
        loader.load<Resampling2D, UniformSampling2D::Ptr, NormalSampling2D::Ptr, ros::NodeHandle&>(resampling_, uniform_sampling_, normal_sampling_, nh_private_);
        if(!resampling_) {
            ROS_ERROR_STREAM("No resampling function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }
        ROS_INFO_STREAM("Loaded resampling algorithm.");
        ROS_INFO_STREAM("[" << resampling_->getName() << "]");
    }
    { /// density estimation
        loader.load<SampleDensity2D, ros::NodeHandle&>(sample_density_, nh_private_);
        if(!sample_density_) {
            ROS_ERROR_STREAM("No sample density estimation function was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }

        ROS_INFO_STREAM("Loaded density estimation function.");
        ROS_INFO_STREAM("[" << sample_density_->getName() << "]");
    }
    { /// scheduling
        loader.load<Scheduler2D, const update_model_map_t&, ros::NodeHandle&>(scheduler_, update_models_, nh_private_);
        if(!scheduler_) {
            ROS_ERROR_STREAM("No scheduler was found!");
            ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
            return false;
        }

        ROS_INFO_STREAM("Loaded scheduler.");
        ROS_INFO_STREAM("[" << scheduler_->getName() << "]");
    }

    //// set up the necessary functions for the particle filter
    {

        const std::string world_frame = nh_private_.param<std::string>("world_frame", "/world");

        auto param_name = [](const std::string &param){return "particle_filter/" + param;};

        const double resampling_threshold_linear      = nh_private_.param<double>(param_name("resampling_threshold_linear"), 0.1);
        const double resampling_threshold_angular     = cslibs_math::common::angle::toRad(nh_private_.param<double>(param_name("resampling_threshold_angular"), 5.0));
        const bool   resampling_threshold_exceed_both = nh_private_.param<bool>(param_name("resampling_threshold_exceed_both"), false);

        const double update_threshold_linear          = nh_private_.param<double>(param_name("update_threshold_linear"), 0.0);
        const double update_threshold_angular         = cslibs_math::common::angle::toRad(nh_private_.param<double>(param_name("update_threshold_angular"), 0.0));
        const bool   update_threshold_exceed_both     = nh_private_.param<bool>(param_name("update_threshold_exceed_both"), false);

        const bool   use_amcl_diff_calculation        = nh_private_.param<bool>(param_name("use_amcl_diff_calculation"), false);

        use_amcl_diff_calculation ?
                    prediction_integrals_.reset(new prediction_integrals_t(PredictionIntegralAMCL2D::Ptr(new PredictionIntegralAMCL2D(resampling_threshold_linear,
                                                                                                                                      resampling_threshold_angular,
                                                                                                                                      resampling_threshold_exceed_both)))) :
                    prediction_integrals_.reset(new prediction_integrals_t(PredictionIntegral2D::Ptr(new PredictionIntegral2D(resampling_threshold_linear,
                                                                                                                              resampling_threshold_angular,
                                                                                                                              resampling_threshold_exceed_both))));
        for(const auto &u : update_models_) {
            use_amcl_diff_calculation ?
                        prediction_integrals_->set(PredictionIntegralAMCL2D::Ptr(new PredictionIntegralAMCL2D(update_threshold_linear,
                                                                                                              update_threshold_angular,
                                                                                                              update_threshold_exceed_both)),
                                                   u.second->getId()) :
                        prediction_integrals_->set(PredictionIntegral2D::Ptr(new PredictionIntegral2D(update_threshold_linear,
                                                                                                      update_threshold_angular,
                                                                                                      update_threshold_exceed_both)),
                                                   u.second->getId());
        }

        const std::size_t sample_size                   = nh_private_.param<int>(param_name("sample_size"), 0);
        const std::size_t minimum_sample_size           = sample_size == 0 ? nh_private_.param<int>(param_name("minimum_sample_size"), 0) : sample_size;
        const std::size_t maximum_sample_size           = sample_size == 0 ? nh_private_.param<int>(param_name("maximum_sample_size"), 0) : sample_size;
        const bool        reset_weights_after_insertion = nh_private_.param<bool>(param_name("reset_weights_after_insertion"), true);
        const bool        reset_weights_to_one          = nh_private_.param<bool>(param_name("reset_weights_to_one"), true);
        const bool        enable_lag_correction         = nh_private_.param<bool>(param_name("enable_lag_correction"), true);
        const bool        reset_all_integrals_on_update = nh_private_.param<bool>(param_name("reset_all_integrals_on_update"), true);

        if(minimum_sample_size == 0) {
            ROS_ERROR_STREAM("Minimum sample size cannot be zero!");
            return false;
        }
        if(maximum_sample_size < minimum_sample_size) {
            ROS_ERROR_STREAM("Maximum sample size cannot be smaller than minimum sample size!");
            return false;
        }

        while (ros::Time::now().toNSec() == 0) { }
        sample_set_.reset(new sample_set_t(world_frame,
                                           cslibs_time::Time(ros::Time::now().toNSec()),
                                           minimum_sample_size,
                                           maximum_sample_size,
                                           sample_density_,
                                           reset_weights_after_insertion,
                                           reset_weights_to_one));
        state_publisher_.reset(new StatePublisher);
        state_publisher_->setup(nh_private_);

        particle_filter_.reset(new smc_t);
        particle_filter_->setup(sample_set_,
                                uniform_sampling_, normal_sampling_,
                                resampling_,
                                state_publisher_,
                                prediction_integrals_,
                                scheduler_,
                                reset_all_integrals_on_update,
                                enable_lag_correction);
    }

    predicition_forwarder_.reset(new PredictionRelay2D(particle_filter_));
    cslibs_plugins_data::DataProvider::Ptr prediction_provider;
    if(!getPredictionProvider(prediction_provider)) {
        ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
        return false;
    }
    predicition_forwarder_->relay(prediction_model_, prediction_provider);

    update_forwarder_.reset(new UpdateRelay2D(particle_filter_));
    update_model_mapping_t update_mapping;
    if(!getUpdateModelProviderMapping(update_mapping)) {
        ROS_ERROR_STREAM("Setup is incomplete and is aborted!");
        return false;
    }
    update_forwarder_->relay(update_mapping);

    initialization_service_pose_    = nh_private_.advertiseService("/muse_mcl_2d/pose_initialization", &MuseMCL2DNode::requestPoseInitialization, this);
    initialization_service_global_  = nh_private_.advertiseService("/muse_mcl_2d/global_initialization", &MuseMCL2DNode::requestGlobalInitialization, this);
    initialization_subscriber_pose_ = nh_private_.subscribe("/initialpose", 1, &MuseMCL2DNode::poseInitialization, this);

    return true;
}

void MuseMCL2DNode::checkPoseInitialization()
{
    auto get_time = []() {
        return cslibs_time::Time(ros::Time::now().toNSec());
    };

    if(nh_private_.hasParam("initialization/pose") &&
            nh_private_.hasParam("initialization/covariance")) {


        std::vector<double> p_v;
        std::vector<double> c_v;
        nh_private_.getParam("initialization/pose", p_v);
        nh_private_.getParam("initialization/covariance",c_v);

        if(p_v.size() != 3) {
            ROS_ERROR_STREAM("The initialization pose is expected to have 3 values [x, y, yaw]");
            return;
        }
        cslibs_math_2d::Pose2d pose = cslibs_math_2d::Pose2d(p_v[0], p_v[1], p_v[2]);

        if(c_v.size() != 9) {
            ROS_ERROR_STREAM("The initialization covariance is expected to have 9 values.");
            return;
        }

        cslibs_math_2d::Covariance3d covariance;
        auto get = [&c_v](std::size_t r, std::size_t c, std::size_t step)
        {
            return c_v[r * step + c];
        };

        covariance(0,0) = get(0,0,3);
        covariance(1,0) = get(1,0,3);
        covariance(2,0) = get(2,0,3);

        covariance(0,1) = get(0,1,3);
        covariance(1,1) = get(1,1,3);
        covariance(2,1) = get(2,1,3);

        covariance(0,2) = get(0,2,3);
        covariance(1,2) = get(1,2,3);
        covariance(2,2) = get(2,2,3);

        particle_filter_->requestStateInitialization(pose, covariance, get_time());
    } else {
        particle_filter_->requestUniformInitialization(get_time());
    }
}

bool MuseMCL2DNode::getUpdateModelProviderMapping(update_model_mapping_t &update_mapping)
{
    for(auto &u : update_models_) {
        const std::string update_model_name = u.first;
        UpdateModel2D::Ptr update_model     = u.second;

        const std::string map_provider_param = update_model_name  + "/map_provider";
        const std::string data_provider_param = update_model_name + "/data_provider";
        const std::string map_provider_name = nh_private_.param<std::string>(map_provider_param, "");
        const std::string data_provider_name = nh_private_.param<std::string>(data_provider_param, "");
        if(map_provider_name == "") {
            ROS_ERROR_STREAM("No map provider name can be resolved for update model '" << update_model_name << "'!");
            return false;
        }
        if(data_provider_name == "") {
            ROS_ERROR_STREAM("No data provider name can be resolved for update model '" << update_model_name << "'!");
            return false;
        }
        if(map_providers_.find(map_provider_name) == map_providers_.end()) {
            ROS_ERROR_STREAM("Map Provider '" << map_provider_name << "' cannot be found for update model '" << update_model_name << "' !");
            return false;
        }
        if(data_providers_.find(data_provider_name) == data_providers_.end()) {
            ROS_ERROR_STREAM("Data Provider '" << data_provider_name << "' cannot be found for update model '" << update_model_name << "' !");
            return false;
        }

        MapProvider2D::Ptr map_provider = map_providers_.at(map_provider_name);
        cslibs_plugins_data::DataProvider::Ptr data_provider = data_providers_.at(data_provider_name);
        update_mapping[update_model] = std::make_pair(data_provider, map_provider);
    }
    return true;
}

bool MuseMCL2DNode::getPredictionProvider(cslibs_plugins_data::DataProvider::Ptr &prediction_provider)
{
    const std::string param_name = prediction_model_->getName() + "/data_provider";
    const std::string provider_name =  nh_private_.param<std::string>(param_name, "");

    if(data_providers_.find(provider_name) == data_providers_.end()) {
        std::cerr << "[MuseAMCLNode]: Could not find data provider '" << provider_name
                  << "' for prediction model '" << prediction_model_->getName()
                  << "' !" << "\n";
        return false;
    }
    prediction_provider = data_providers_.at(provider_name);
    return true;
}
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl_2d");

    muse_mcl_2d::MuseMCL2DNode node;
    if(node.setup()) {
        ROS_INFO_STREAM("Node is set up and ready to start!");
        node.start();
    } else {
        ROS_ERROR_STREAM("Could not set up the node!");
    }
    return 0;
}

