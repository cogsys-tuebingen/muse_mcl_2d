#ifndef MUSE_MCL_2D_GRIDMAPS_BINARY_GRIDMAP_HPP
#define MUSE_MCL_2D_GRIDMAPS_BINARY_GRIDMAP_HPP

#include <muse_mcl_2d/map/map_2d.hpp>

#include <cslibs_time/stamped.hpp>
#include <cslibs_math/common/framed.hpp>
#include <cslibs_gridmaps/static_maps/binary_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class BinaryGridmap : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<BinaryGridmap>;

    BinaryGridmap(const cslibs_gridmaps::static_maps::BinaryGridmap::Ptr &map,
                  const std::string frame_id);

    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const cslibs_math_2d::Pose2d &p) const override;
    cslibs_gridmaps::static_maps::BinaryGridmap::Ptr& data();
    cslibs_gridmaps::static_maps::BinaryGridmap::Ptr const & data() const;

private:
    cslibs_gridmaps::static_maps::BinaryGridmap::Ptr data_;

};
}

#endif // MUSE_MCL_2D_GRIDMAPS_BINARY_GRIDMAP_HPP
