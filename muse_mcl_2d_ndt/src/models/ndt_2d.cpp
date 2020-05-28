#include "ndt_2d.h"

#include <cslibs_plugins_data/types/laserscan.hpp>

#include <cslibs_ndt_2d/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/merge.hpp>

#include <muse_mcl_2d_ndt/maps/flat_gridmap_2d.h>

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::NDT2D, muse_mcl_2d::UpdateModel2D)

namespace muse_mcl_2d_ndt {
  void NDT2D::apply(const data_t::ConstPtr          &data,
                    const std::shared_ptr<state_space_t const>   &map,
                    sample_set_t::weight_iterator_t  set)
  {
      using laserscan_t = cslibs_plugins_data::types::Laserscan2d;
      using transform_t = muse_mcl_2d::StateSpaceDescription2D::transform_t;
      using state_t     = muse_mcl_2d::StateSpaceDescription2D::state_t;
      using point_t     = muse_mcl_2d::StateSpaceDescription2D::state_space_boundary_t;

      if (!map->isType<FlatGridmap2D>() || !data->isType<laserscan_t>())
          return;

      const FlatGridmap2D::map_t &gridmap    = *(map->as<FlatGridmap2D>().data());
      const laserscan_t          &laser_data = data->as<laserscan_t>();

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

      const laserscan_t::rays_t rays = laser_data.getRays();
      cslibs_ndt_2d::dynamic_maps::Gridmap<double>::Ptr local_gridmap(new cslibs_ndt_2d::dynamic_maps::Gridmap<double>(gridmap.getResolution()));
      for(const auto &r : rays) {
          if(r.valid()) {
              point_t p(std::cos(r.angle) * r.range,
                                        std::sin(r.angle) * r.range);
              local_gridmap->insert(p);
          }
      }

      FlatGridmap2D::map_t::Ptr local_flattened_map(cslibs_ndt_2d::conversion::merge(local_gridmap));

      for(auto it = set.begin() ; it != set.end() ; ++it) {
          auto update = [&it, &gridmap, &m_T_w, &b_T_l, this](const FlatGridmap2D::map_t::index_t&,
                                                              const FlatGridmap2D::map_t::distribution_t &data)
          {
              const state_t m_T_l = m_T_w * it.state() * b_T_l;
              const Eigen::Matrix2d rot = m_T_l.getEigenRotation();
              const Eigen::Matrix2d rot_t = rot.transpose();
              double p = 1.0;

              const cslibs_math::statistics::Distribution<double,2, 3> &dl = data.data();
              if(dl.getN() >= 3) {
                  const Eigen::Vector2d m = (m_T_l * point_t(dl.getMean())).data();
                  const FlatGridmap2D::map_t::distribution_t *db = gridmap.get(point_t(m));
                  if(db) {
                      const cslibs_math::statistics::Distribution<double,2, 3> &d = db->data();

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
