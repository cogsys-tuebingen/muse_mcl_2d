#ifndef MUSE_MCL_2D_GRIDMAPS_DISTANCE_GRIDMAP_HPP
#define MUSE_MCL_2D_GRIDMAPS_DISTANCE_GRIDMAP_HPP

#include <muse_mcl_2d/map/map_2d.hpp>

#include <cslibs_time/stamped.hpp>
#include <cslibs_gridmaps/static_maps/distance_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class DistanceGridmap : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<DistanceGridmap>;

    DistanceGridmap(const cslibs_gridmaps::static_maps::DistanceGridmap::Ptr &map,
                    const std::string frame_id);

    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const cslibs_math_2d::Pose2d &p_w) const override;
    cslibs_gridmaps::static_maps::DistanceGridmap::Ptr& data();
    cslibs_gridmaps::static_maps::DistanceGridmap::Ptr const & data() const;

private:
    cslibs_gridmaps::static_maps::DistanceGridmap::Ptr data_;

};
}
#endif // MUSE_MCL_2D_GRIDMAPS_DISTANCE_GRIDMAP_HPP
