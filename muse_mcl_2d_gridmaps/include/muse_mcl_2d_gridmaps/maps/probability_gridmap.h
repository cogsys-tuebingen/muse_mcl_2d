#ifndef MUSE_MCL_2D_GRIDMAPS_PROBABILITY_GRIDMAP_HPP
#define MUSE_MCL_2D_GRIDMAPS_PROBABILITY_GRIDMAP_HPP
#include <muse_mcl_2d/map/map_2d.hpp>

#include <cslibs_time/stamped.hpp>
#include <cslibs_math/common/framed.hpp>
#include <cslibs_gridmaps/static_maps/probability_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class ProbabilityGridmap : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<ProbabilityGridmap>;

    ProbabilityGridmap(const cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr &map,
                       const std::string frame_id);
    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const cslibs_math_2d::Pose2d &p_w) const override;
    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr& data();
    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr const & data() const;
private:
    cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr data_;

};
}
#endif // PROBABILITY_GRIDMAP_HPP
