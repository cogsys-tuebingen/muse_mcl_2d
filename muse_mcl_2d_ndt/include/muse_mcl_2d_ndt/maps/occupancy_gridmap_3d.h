#ifndef MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_H
#define MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_ndt_3d/dynamic_maps/occupancy_gridmap.hpp>

namespace muse_mcl_2d_ndt {
class OccupancyGridmap3d : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<OccupancyGridmap3d>;
    using map_t = cslibs_ndt_3d::dynamic_maps::OccupancyGridmap<double>;
    using state_t = muse_mcl_2d::StateSpaceDescription2D::state_t;
    using point_t = muse_mcl_2d::StateSpaceDescription2D::state_space_boundary_t;

    OccupancyGridmap3d(const map_t::Ptr &map,
                       const std::string frame_id);

    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const state_t &p) const override;
    map_t::Ptr& data();
    map_t::Ptr const& data() const;

private:
    map_t::Ptr data_;
};
}

#endif // MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_3D_H
