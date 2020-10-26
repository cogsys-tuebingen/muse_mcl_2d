#include <muse_mcl_2d_ndt/models/occupancy_gridmap_3d_likelihood_field_model.h>

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>
#include <muse_mcl_2d_ndt/maps/occupancy_gridmap_3d.h>
#include <muse_mcl_2d_ndt/utility/pointcloud_3d_histogram.hpp>

#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::OccupancyGridmap3dLikelihoodFieldModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
OccupancyGridmap3dLikelihoodFieldModel::OccupancyGridmap3dLikelihoodFieldModel()
{
}

void OccupancyGridmap3dLikelihoodFieldModel::apply(const data_t::ConstPtr         &data,
                                                   const std::shared_ptr<state_space_t const>  &map,
                                                   sample_set_t::weight_iterator_t set)
{
    using pointcloud_t   = cslibs_plugins_data::types::Pointcloud3d;
    using transform_t    = cslibs_math_3d::Transform3d;
    using point_t        = cslibs_math_3d::Point3d;
    using distribution_t = typename OccupancyGridmap3d::map_t::distribution_t::distribution_t;

    if (!map->isType<OccupancyGridmap3d>() || !data->isType<pointcloud_t>())
        return;

    const OccupancyGridmap3d::map_t &gridmap    = *(map->as<OccupancyGridmap3d>().data());
    const pointcloud_t              &cloud_data = data->as<pointcloud_t>();
    const cslibs_math_3d::Pointcloud3d::ConstPtr &cloud_points = cloud_data.points();

    /// cloud to base transform
    transform_t b_T_s, m_T_w;
    if (!tf_->lookupTransform(robot_base_frame_,
                              cloud_data.frame(),
                              ros::Time(cloud_data.timeFrame().end.seconds()),
                              b_T_s,
                              tf_timeout_))
        return;
    if (!tf_->lookupTransform(world_frame_,
                              map->getFrame(),
                              ros::Time(cloud_data.timeFrame().end.seconds()),
                              m_T_w,
                              tf_timeout_))
        return;

    // mixture distribution entries
    const double bundle_resolution_inv = 1.0 / gridmap.getBundleResolution();
    auto to_bundle_index = [&bundle_resolution_inv](const point_t &p) {
        return std::array<int, 3>({{static_cast<int>(std::floor(p(0) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(1) * bundle_resolution_inv)),
                                    static_cast<int>(std::floor(p(2) * bundle_resolution_inv))}});
    };
    auto likelihood = [this](const point_t &p,
                             const typename distribution_t::Ptr &d,
                             const double &inv_occ) {
        auto apply = [&p, &d, &inv_occ, this](){
            const auto &q         = p.data() - d->getMean();
            const double exponent = -0.5 * d_ * inv_occ * double(q.transpose() * d->getInformationMatrix() * q);
            const double e        = std::exp(exponent);
            return std::isnormal(e) ? e : 0.0;
        };
        return !d ? 0.0 : apply();
    };
    auto occupancy_likelihood = [this, &likelihood](const point_t &p,
                                                    const cslibs_ndt::OccupancyDistribution<double,3>* d) {
        double occ = d ? d->getOccupancy(inverse_model_) : 0.0;
        double ndt = d ? occ * likelihood(p, d->getDistribution(), 1.0 - occ) : 0.0;

        return ndt;
    };
    auto bundle_likelihood = [&gridmap, &to_bundle_index, &occupancy_likelihood](const point_t &p) {
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

    if (histogram_resolution_ > 0.0) {
        utility_pcl::kd_tree_t  histogram;
        utility_pcl::Indexation index(histogram_resolution_);

        const std::size_t points_size = cloud_points->size();
        for (std::size_t i = 0; i < points_size; ++i) {
            const auto &p = cloud_points->at(i);
            if (p.isNormal())
                histogram.insert(index.create(p), utility_pcl::Data(i, p));
        }

        std::vector<std::size_t> cloud_indices;
        utility_pcl::getRepresentativePoints(histogram, *cloud_points, cloud_indices);

        for (auto it = set.begin(); it != set.end(); ++it) {
            transform_t it_s(it.state().tx(), it.state().ty(), 0, it.state().yaw()); /// cloud camera pose in map coordinates
            transform_t m_T_s = m_T_w * it_s * b_T_s;
            double p = 1.0;
            for (const std::size_t i : cloud_indices) {
                const auto &point = cloud_points->at(i);
                const point_t map_point = m_T_s * point;
                /// TODO: what if map origin is not identity?
                p += map_point.isNormal() ? pow3(p_hit_ * bundle_likelihood(map_point) + p_rand_) : pow3(p_max_);
            }
            *it *= p;
        }
    } else {
        const std::size_t points_size = cloud_points->size();
        const std::size_t points_step = std::max(1ul, (points_size - 1) / (max_points_ - 1));
        for (auto it = set.begin() ; it != set.end() ; ++it) {
            transform_t it_s(it.state().tx(), it.state().ty(), 0, it.state().yaw()); /// cloud camera pose in map coordinates
            transform_t m_T_s = m_T_w * it_s * b_T_s;
            double p = 1.0;
            for (std::size_t i = 0 ; i < points_size ;  i+= points_step) {
                const auto &point = cloud_points->at(i);
                const point_t map_point = m_T_s * point;
                /// TODO: what if map origin is not identity?
                p += map_point.isNormal() ? pow3(p_hit_ * bundle_likelihood(map_point) + p_rand_) : pow3(p_max_);
            }
            *it *= p;
        }
    }
}

void OccupancyGridmap3dLikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_ = nh.param(param_name("max_points"), 100);
    d_          = nh.param(param_name("d"), 1.0);
    p_rand_     = nh.param(param_name("p_rand"), 0.2);
    p_max_      = nh.param(param_name("p_max"), 0.0);
    p_hit_      = nh.param(param_name("p_hit"), 0.8);

    occupied_threshold_         = nh.param(param_name("occupied_threshold"), 0.169);
    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel<double>(prob_prior, prob_free, prob_occupied));

    histogram_resolution_  = nh.param(param_name("histogram_resolution"), 0.0);
}
}
