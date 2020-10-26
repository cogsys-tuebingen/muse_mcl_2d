#include <muse_mcl_2d_ndt/models/gridmap_2d_likelihood_field_model.h>
#include <muse_mcl_2d_ndt/maps/gridmap_2d.h>
#include <muse_mcl_2d_ndt/utility/laser_histogram.hpp>

#include <cslibs_plugins_data/types/laserscan.hpp>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::Gridmap2dLikelihoodFieldModel, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
Gridmap2dLikelihoodFieldModel::Gridmap2dLikelihoodFieldModel()
{
}

void Gridmap2dLikelihoodFieldModel::apply(const data_t::ConstPtr         &data,
                                          const std::shared_ptr<state_space_t const>  &map,
                                          sample_set_t::weight_iterator_t set)
{
    using distribution_t = typename Gridmap2d::map_t::distribution_t::distribution_t;
    using laserscan_t = cslibs_plugins_data::types::Laserscan2d;

    if (!map->isType<Gridmap2d>() || !data->isType<laserscan_t>())
        return;

    const Gridmap2d::map_t    &gridmap    = *(map->as<Gridmap2d>().data());
    const laserscan_t         &laser_data = data->as<laserscan_t>();
    const laserscan_t::rays_t &laser_rays = laser_data.getRays();

    /// laser to base transform
    transform_t b_T_l, m_T_w;
    if (!tf_->lookupTransform(robot_base_frame_,
                              laser_data.frame(),
                              ros::Time(laser_data.timeFrame().end.seconds()),
                              b_T_l,
                              tf_timeout_))
        return;
    if (!tf_->lookupTransform(world_frame_,
                              map->getFrame(),
                              ros::Time(laser_data.timeFrame().end.seconds()),
                              m_T_w,
                              tf_timeout_))
        return;

    /// evaluation functions
    auto likelihood = [this](const point_t &p,
                             const distribution_t &d) {
        const auto &q         = p.data() - d.getMean();
        const double exponent = -0.5 * d_ * double(q.transpose() * d.getInformationMatrix() * q);
        const double e        = std::exp(exponent);
        // for experiment put sth similar to a laplace distribution here
        // const auto support_points = d.getN();
        // const auto scale = [support_points]() {
        //     return 1.0 - std::exp(-0.25 * static_cast<double>(support_points));
        // };
        return std::isnormal(e) ? e : 0.0;
    };
    auto bundle_likelihood = [&gridmap, &likelihood](const point_t &p) {
        const auto *bundle = gridmap.getDistributionBundle(p);
        assert(bundle != nullptr);
        return 0.25 * (likelihood(p, *(bundle->at(0))) +
                       likelihood(p, *(bundle->at(1))) +
                       likelihood(p, *(bundle->at(2))) +
                       likelihood(p, *(bundle->at(3))));
    };

    auto pow3 = [](const double& x) {
        return x*x*x;
    };
    const laserscan_t::rays_t &rays = laser_data.getRays();
    const std::size_t rays_size = rays.size();
    const double range_min = laser_data.getLinearMin();
    const double range_max = laser_data.getLinearMax();
    const double angle_min = laser_data.getAngularMin();
    const double angle_max = laser_data.getAngularMax();

    auto valid = [range_min, range_max, angle_min, angle_max](const laserscan_t::Ray &r){
        return r.valid()
            && r.angle >= angle_min && r.angle <= angle_max
            && r.range >= range_min && r.range <= range_max;
    };

    if (histogram_resolution_ > 0.0) {
        utility_ray::kd_tree_t  histogram;
        utility_ray::Indexation index(histogram_resolution_);

        const std::size_t size = rays.size();
        for (std::size_t i = 0 ; i < size ; ++i) {
           const auto &r = rays[i];
           if (valid(r))
              histogram.insert(index.create(r), utility_ray::Data(i, r));
        }

        std::vector<std::size_t> ray_indices;
        utility_ray::getRepresentativeRays(histogram, rays, ray_indices);

        for (auto it = set.begin() ; it != set.end() ; ++it) {
            const state_t m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
            double p = 1.0;
            for (const std::size_t ri : ray_indices) {
                const auto &ray = laser_rays[ri];
                const point_t map_point = m_T_l * ray.end_point;
                p += ray.valid() && map_point.isNormal() ? pow3(p_hit_ * bundle_likelihood(map_point) + p_rand_) : pow3(p_max_);
            }
            *it *= p;
        }
    } else {
        const std::size_t ray_step  = std::max(1ul, (rays_size - 1) / (max_points_ - 1));
        for (auto it = set.begin() ; it != set.end() ; ++it) {
            const state_t m_T_l = m_T_w * it.state() * b_T_l; /// laser scanner pose in map coordinates
            double p = 1.0;
            for (std::size_t i = 0 ; i < rays_size ; i+= ray_step) {
                const auto &ray = laser_rays[i];
                const point_t map_point = m_T_l * ray.end_point;
                p += ray.valid() && map_point.isNormal() ? pow3(bundle_likelihood(map_point) + p_rand_) : pow3(p_max_);
            }
            *it *= p;
        }
    }
}

void Gridmap2dLikelihoodFieldModel::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    max_points_            = nh.param(param_name("max_points"), 100);
    d_                     = nh.param(param_name("d"), 1.0);
    p_rand_                = nh.param(param_name("p_rand"), 0.2);
    p_max_                 = nh.param(param_name("p_max"), 0.0);
    p_hit_                 = nh.param(param_name("p_hit"), 0.8);

    histogram_resolution_  = nh.param(param_name("histogram_resolution"), 0.0);
}
}

