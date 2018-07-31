#include <muse_mcl_2d_ndt/models/occupancy_gridmap_3d_likelihood_field_model.h>

#include <cslibs_plugins_data/types/pointcloud.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>

#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::OccupancyGridmap3dLikelihoodFieldModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
OccupancyGridmap3dLikelihoodFieldModel::OccupancyGridmap3dLikelihoodFieldModel()
{
}

void OccupancyGridmap3dLikelihoodFieldModel::apply(const data_t::ConstPtr &data,
                                          const state_space_t::ConstPtr   &map,
                                          sample_set_t::weight_iterator_t set)
{
    if (!map->isType<OccupancyGridmap3d>() || !data->isType<cslibs_plugins_data::types::Pointcloud>())
        return;

    const cslibs_ndt_3d::dynamic_maps::OccupancyGridmap &gridmap       = *(map->as<OccupancyGridmap3d>().data());
    const cslibs_plugins_data::types::Pointcloud                &stereo_data   = data->as<cslibs_plugins_data::types::Pointcloud>();
    const cslibs_math_3d::Pointcloud3d::Ptr             &stereo_points = stereo_data.getPoints();

    /// stereo to base transform
    tf::Transform b_T_s, m_T_w;
    if (!tf_->lookupTransform(robot_base_frame_,
                              stereo_data.getFrame(),
                              ros::Time(stereo_data.getTimeFrame().end.seconds()),
                              b_T_s,
                              tf_timeout_))
        return;
    if (!tf_->lookupTransform(world_frame_,
                              map->getFrame(),
                              ros::Time(stereo_data.getTimeFrame().end.seconds()),
                              m_T_w,
                              tf_timeout_))
        return;
    cslibs_math_3d::Transform3d b_T_s_3d = cslibs_math_ros::tf::conversion_3d::from(b_T_s);
    cslibs_math_3d::Transform3d m_T_w_3d = cslibs_math_ros::tf::conversion_3d::from(m_T_w);

    const std::size_t points_size = stereo_points->size();
    const std::size_t points_step = std::max(1ul, (points_size - 1) / (max_points_ - 1));

    // mixture distribution entries
    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const cslibs_math_3d::Point3d &p) {
        return std::array<int, 3>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(2) * bundle_resolution_inv))}});
    };
    auto likelihood = [this](const cslibs_math_3d::Point3d &p,
                             const cslibs_math::statistics::Distribution<3, 3>::Ptr &d,
                             const double &inv_occ) {
        auto apply = [&p, &d, &inv_occ, this](){
            const auto &q         = p.data() - d->getMean();
            const double exponent = -0.5 * d2_ * inv_occ * double(q.transpose() * d->getInformationMatrix() * q);
            const double e        = d1_ * std::exp(exponent);
            return std::isnormal(e) ? e : 0.0;
        };
        return !d ? 0.0 : apply();
    };
    auto occupancy_likelihood = [this, &likelihood](const cslibs_math_3d::Point3d &p,
                                                    const cslibs_ndt::OccupancyDistribution<3>* d) {
        double occ = d ? d->getOccupancy(inverse_model_) : 0.0;
        double ndt = d ? occ * likelihood(p, d->getDistribution(), 1.0 - occ) : 0.0;

        return ndt;
    };
    auto bundle_likelihood = [this, &gridmap, &to_bundle_index, &occupancy_likelihood](const cslibs_math_3d::Point3d &p) {
        const auto &bundle = gridmap.getDistributionBundle(to_bundle_index(p));
        return 0.125 * (occupancy_likelihood(p, bundle->at(0)) +
                        occupancy_likelihood(p, bundle->at(1)) +
                        occupancy_likelihood(p, bundle->at(2)) +
                        occupancy_likelihood(p, bundle->at(3)) +
                        occupancy_likelihood(p, bundle->at(4)) +
                        occupancy_likelihood(p, bundle->at(5)) +
                        occupancy_likelihood(p, bundle->at(6)) +
                        occupancy_likelihood(p, bundle->at(7)));
    };

    auto pow3 = [](const double& x) {
        return x*x*x;
    };
    for (auto it = set.begin() ; it != set.end() ; ++it) {
        cslibs_math_3d::Transform3d it_s(it.state().tx(), it.state().ty(), 0, it.state().yaw()); /// stereo camera pose in map coordinates
        cslibs_math_3d::Transform3d m_T_s = m_T_w_3d * it_s * b_T_s_3d;
        double p = 1.0;
        for (std::size_t i = 0 ; i < points_size ;  i+= points_step) {
            const auto &point = stereo_points->at(i);
            const cslibs_math_3d::Point3d map_point = m_T_s * point;
            /// TODO: what if map origin is not identity?
            p += map_point.isNormal() ? pow3(bundle_likelihood(map_point)) : 0.0;
        }
        *it *= p;
    }
}

void OccupancyGridmap3dLikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_ = nh.param(param_name("max_points"), 100);
    d1_         = nh.param(param_name("d1"), 0.95);
    d2_         = nh.param(param_name("d2"), 0.05);

    occupied_threshold_         = nh.param(param_name("occupied_threshold"), 0.196);
    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel(prob_prior, prob_free, prob_occupied));
}
}
