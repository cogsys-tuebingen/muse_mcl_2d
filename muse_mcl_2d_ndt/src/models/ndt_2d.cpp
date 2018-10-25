#include "ndt_2d.h"

#include <cslibs_plugins_data/types/laserscan.hpp>

#include <cslibs_ndt_2d/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/merge.hpp>

#include <muse_mcl_2d_ndt/maps/flat_gridmap_2d.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::NDT2D, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
  void NDT2D::apply(const data_t::ConstPtr          &data,
                    const state_space_t::ConstPtr   &map,
                    sample_set_t::weight_iterator_t  set)
  {
    if (!map->isType<FlatGridmap2D>() || !data->isType<cslibs_plugins_data::types::Laserscan>())
      return;

    const cslibs_ndt_2d::static_maps::mono::Gridmap &gridmap    = *(map->as<FlatGridmap2D>().data());
    const cslibs_plugins_data::types::Laserscan     &laser_data = data->as<cslibs_plugins_data::types::Laserscan>();

    cslibs_math_2d::Transform2d b_T_l, m_T_w;
    if (!tf_->lookupTransform(robot_base_frame_,
                              laser_data.getFrame(),
                              ros::Time(laser_data.getTimeFrame().end.seconds()),
                              b_T_l,
                              tf_timeout_))
      return;
    if (!tf_->lookupTransform(world_frame_,
                              map->getFrame(),
                              ros::Time(laser_data.getTimeFrame().end.seconds()),
                              m_T_w,
                              tf_timeout_))
      return;

    const cslibs_plugins_data::types::Laserscan::rays_t rays = laser_data.getRays();
    cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr local_gridmap(new cslibs_ndt_2d::dynamic_maps::Gridmap(gridmap.getResolution()));
    for(const auto &r : rays) {
      if(r.valid()) {
        cslibs_math_2d::Point2d p(std::cos(r.angle) * r.range,
                                  std::sin(r.angle) * r.range);
        local_gridmap->insert(p);
      }
    }

    cslibs_ndt_2d::static_maps::mono::Gridmap::Ptr local_flattened_map(cslibs_ndt_2d::conversion::merge(local_gridmap));

    for(auto it = set.begin() ; it != set.end() ; ++it) {
      auto update = [&it, &gridmap, &m_T_w, &b_T_l, this](const cslibs_ndt_2d::static_maps::mono::Gridmap::index_t&,
          const cslibs_ndt_2d::static_maps::mono::Gridmap::distribution_t &data)
      {
        const cslibs_math_2d::Pose2d m_T_l = m_T_w * it.state() * b_T_l;
        const Eigen::Matrix2d rot = m_T_l.getEigenRotation();
        const Eigen::Matrix2d rot_t = rot.transpose();
        double p = 1.0;

        const cslibs_math::statistics::Distribution<2, 3> &dl = data.data();
        if(dl.getN() >= 3) {
          const Eigen::Vector2d m = (m_T_l * cslibs_math_2d::Point2d(dl.getMean())).data();
          const cslibs_ndt_2d::static_maps::mono::Gridmap::distribution_t *db = gridmap.get(cslibs_math_2d::Point2d(m));
          if(db) {
            const cslibs_math::statistics::Distribution<2, 3> &d = db->data();

            if(d.getN() >= 3) {
              const Eigen::Matrix2d c =  rot * dl.getCovariance() * rot_t;
              const auto cc = c + d.getCovariance();
              const auto dm = d.getMean() - m;
              if(cc.determinant() != 0.0) {
                const auto icc = cc.inverse();
                const double l = dm.dot(icc * dm);
                if(std::isnormal(l)) {
                  std::cerr << l << "\n";
                  p += 0.1 + d1_ * std::exp( -0.5 * d2_ * l);
                }
              }
            }
          }
        }
        *it *= p;
      };
      local_flattened_map->traverse(update);
    }
  }

  void NDT2D::doSetup(ros::NodeHandle &nh)
  {
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    d1_         = nh.param(param_name("d1"), 0.95);
    d2_         = nh.param(param_name("d2"), 0.05);
  }
}
