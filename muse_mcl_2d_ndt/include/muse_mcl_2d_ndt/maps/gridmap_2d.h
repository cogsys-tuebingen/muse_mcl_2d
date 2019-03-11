#ifndef MUSE_MCL_2D_NDT_GRIDMAP_2D_H
#define MUSE_MCL_2D_NDT_GRIDMAP_2D_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_ndt_2d/dynamic_maps/gridmap.hpp>

namespace muse_mcl_2d_ndt {
class Gridmap2d : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<Gridmap2d>;
    using map_t = cslibs_ndt_2d::dynamic_maps::Gridmap<double>;
    using state_t = muse_mcl_2d::StateSpaceDescription2D::state_t;

    Gridmap2d(const map_t::Ptr &map,
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

#endif // MUSE_MCL_2D_NDT_GRIDMAP_2D_H
