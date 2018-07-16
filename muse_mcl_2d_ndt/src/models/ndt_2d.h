#ifndef MUSE_MCL_2D_NDT_2D_H
#define MUSE_MCL_2D_NDT_2D_H

#include <muse_mcl_2d/update/update_model_2d.hpp>

namespace muse_mcl_2d_ndt {
class NDT2D : public muse_mcl_2d::UpdateModel2D
{
public:
  NDT2D() = default;

  void apply(const data_t::ConstPtr          &data,
             const state_space_t::ConstPtr   &map,
             sample_set_t::weight_iterator_t  set) override;

private:
  double      d1_;
  double      d2_;

  virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // MUSE_MCL_2D_NDT_2D_H
