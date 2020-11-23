#ifndef MUSE_MCL_2D_GRIDMAPS_PROBABILITY_GRIDMAP_HPP
#define MUSE_MCL_2D_GRIDMAPS_PROBABILITY_GRIDMAP_HPP
#include <muse_mcl_2d/map/map_2d.hpp>

#include <cslibs_time/stamped.hpp>
#include <cslibs_gridmaps/static_maps/probability_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class ProbabilityGridmap : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<ProbabilityGridmap>;
    using map_t = cslibs_gridmaps::static_maps::ProbabilityGridmap<double,double>;
    using state_t = muse_smc::traits::State<muse_mcl_2d::Hypothesis2D>::type;

    ProbabilityGridmap(const map_t::Ptr &map,
                       const std::string frame_id);
    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const state_t &p_w) const override;
    map_t::Ptr& data();
    map_t::Ptr const & data() const;

private:
    map_t::Ptr data_;
};
}

#endif // PROBABILITY_GRIDMAP_HPP
