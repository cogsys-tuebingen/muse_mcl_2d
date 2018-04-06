#ifndef MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_2D_H
#define MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_2D_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_ndt_2d/dynamic_maps/occupancy_gridmap.hpp>

namespace muse_mcl_2d_ndt {
class OccupancyGridmap2d : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<OccupancyGridmap2d>;

    OccupancyGridmap2d(const cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr &map,
                     const std::string frame_id);

    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const cslibs_math_2d::Pose2d &p) const override;
    cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr& data();
    cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr const& data() const;

private:
    cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr data_;
};
}

#endif // MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_2D_H
