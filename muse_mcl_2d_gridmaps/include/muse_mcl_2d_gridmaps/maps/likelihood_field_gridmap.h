#ifndef MUSE_MCL_2D_GRIDMAPS_LIKELIHOOD_FIELD_GRIDMAP_HPP
#define MUSE_MCL_2D_GRIDMAPS_LIKELIHOOD_FIELD_GRIDMAP_HPP
#include <muse_mcl_2d/map/map_2d.hpp>

#include <cslibs_time/stamped.hpp>
#include <cslibs_gridmaps/static_maps/likelihood_field_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class LikelihoodFieldGridmap : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<LikelihoodFieldGridmap>;

    LikelihoodFieldGridmap(const cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr &map,
                                  const std::string frame_id);
    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const cslibs_math_2d::Pose2d &p_w) const override;
    cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr& data();
    cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr const & data() const;
private:
    cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr data_;

};
}
#endif // LIKELIHOOD_FIELD_GRIDMAP_HPP
