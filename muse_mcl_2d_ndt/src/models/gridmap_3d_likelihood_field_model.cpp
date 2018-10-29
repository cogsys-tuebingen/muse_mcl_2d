#include <muse_mcl_2d_ndt/models/gridmap_3d_likelihood_field_model.h>

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <muse_mcl_2d_ndt/maps/gridmap_3d.h>

#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::Gridmap3dLikelihoodFieldModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
Gridmap3dLikelihoodFieldModel::Gridmap3dLikelihoodFieldModel()
{
}

void Gridmap3dLikelihoodFieldModel::apply(const data_t::ConstPtr          &data,
                                          const state_space_t::ConstPtr   &map,
                                          sample_set_t::weight_iterator_t set)
{
    if (!map->isType<Gridmap3d>() || !data->isType<cslibs_plugins_data::types::Pointcloud3d>())
        return;

    const cslibs_ndt_3d::dynamic_maps::Gridmap      &gridmap       = *(map->as<Gridmap3d>().data());
    const cslibs_plugins_data::types::Pointcloud3d  &stereo_data   = data->as<cslibs_plugins_data::types::Pointcloud3d>();
    const cslibs_math_3d::Pointcloud3d::ConstPtr    &stereo_points = stereo_data.points();

    /// stereo to base transform
    cslibs_math_3d::Transform3d b_T_s, m_T_w;
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
                             const cslibs_math::statistics::Distribution<3, 3> &d) {
        const auto &q         = p.data() - d.getMean();
        const double exponent = -0.5 * d2_ * double(q.transpose() * d.getInformationMatrix() * q);
        const double e        = d1_ * std::exp(exponent);
        return std::isnormal(e) ? e : 0.0;
    };
    auto bundle_likelihood = [&gridmap, &to_bundle_index, &likelihood](const cslibs_math_3d::Point3d &p) {
        const auto &bundle = gridmap.getDistributionBundle(to_bundle_index(p));
        return 0.125 * (likelihood(p, bundle->at(0)->data()) +
                        likelihood(p, bundle->at(1)->data()) +
                        likelihood(p, bundle->at(2)->data()) +
                        likelihood(p, bundle->at(3)->data()) +
                        likelihood(p, bundle->at(4)->data()) +
                        likelihood(p, bundle->at(5)->data()) +
                        likelihood(p, bundle->at(6)->data()) +
                        likelihood(p, bundle->at(7)->data()));
    };

    auto pow3 = [](const double& x) {
        return x*x*x;
    };
    for (auto it = set.begin() ; it != set.end() ; ++it) {
        cslibs_math_3d::Transform3d it_s(it.state().tx(), it.state().ty(), 0, it.state().yaw()); /// stereo camera pose in map coordinates
        cslibs_math_3d::Transform3d m_T_s = m_T_w * it_s * b_T_s;
        double p = 1.0;
        for (std::size_t i = 0 ; i < points_size ;  i+= points_step) {
            const auto &point = stereo_points->at(i);
            const cslibs_math_3d::Point3d map_point = m_T_s * point;
            p += map_point.isNormal() ? pow3(bundle_likelihood(map_point)) : 0.0;
        }
        *it *= p;
    }
}

void Gridmap3dLikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_ = nh.param(param_name("max_points"), 100);
    d1_         = nh.param(param_name("d1"), 0.95);
    d2_         = nh.param(param_name("d2"), 0.05);
}
}
