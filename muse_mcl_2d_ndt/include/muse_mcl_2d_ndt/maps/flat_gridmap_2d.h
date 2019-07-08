#ifndef FLAT_GRIDMAP_2D_H
#define FLAT_GRIDMAP_2D_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_ndt_2d/static_maps/mono_gridmap.hpp>

namespace muse_mcl_2d_ndt {
class FlatGridmap2D : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<FlatGridmap2D>;
    using map_t = cslibs_ndt_2d::static_maps::mono::Gridmap<double>;
    using state_t = muse_mcl_2d::Sample2D::state_t;

    FlatGridmap2D(const map_t::Ptr &map,
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
#endif // FLAT_GRIDMAP_2D_H
