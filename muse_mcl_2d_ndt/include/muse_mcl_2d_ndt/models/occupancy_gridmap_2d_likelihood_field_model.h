#ifndef MUSE_MCL_2D_OCCUPANCY_GRIDMAP_2D_LIKELIHOOD_FIELD_MODEL_H
#define MUSE_MCL_2D_OCCUPANCY_GRIDMAP_2D_LIKELIHOOD_FIELD_MODEL_H

#include <muse_mcl_2d/update/update_model_2d.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

namespace muse_mcl_2d_ndt {
class OccupancyGridmap2dLikelihoodFieldModel : public muse_mcl_2d::UpdateModel2D
{
public:
    OccupancyGridmap2dLikelihoodFieldModel();

    virtual void apply(const data_t::ConstPtr         &data,
                       const std::shared_ptr<state_space_t const>  &map,
                       sample_set_t::weight_iterator_t set) override;

protected:
    std::size_t max_points_;
    double      d_, p_rand_, p_max_, p_hit_;
    double      occupied_threshold_;
    double      histogram_resolution_;
    cslibs_gridmaps::utility::InverseModel<double>::Ptr inverse_model_;

    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // MUSE_MCL_2D_OCCUPANCY_GRIDMAP_2D_LIKELIHOOD_FIELD_MODEL_H
