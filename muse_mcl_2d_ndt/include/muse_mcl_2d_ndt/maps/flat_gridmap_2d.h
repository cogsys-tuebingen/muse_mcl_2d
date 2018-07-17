#ifndef FLAT_GRIDMAP_2D_H
#define FLAT_GRIDMAP_2D_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_ndt_2d/static_maps/flat_gridmap.hpp>

namespace muse_mcl_2d_ndt {
class FlatGridmap2D : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<FlatGridmap2D>;

    FlatGridmap2D(const cslibs_ndt_2d::static_maps::flat::Gridmap::Ptr &map,
                  const std::string frame_id);

    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const cslibs_math_2d::Pose2d &p) const override;
    cslibs_ndt_2d::static_maps::flat::Gridmap::Ptr& data();
    cslibs_ndt_2d::static_maps::flat::Gridmap::Ptr const& data() const;

private:
    cslibs_ndt_2d::static_maps::flat::Gridmap::Ptr data_;
};
}
#endif // FLAT_GRIDMAP_2D_H
