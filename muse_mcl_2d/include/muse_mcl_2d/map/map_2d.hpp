#ifndef MAP_2D_HPP
#define MAP_2D_HPP

#include <muse_smc/state_space/state_space.hpp>
#include <muse_mcl_2d/state_space/state_space_description_2d.hpp>

namespace muse_mcl_2d {
using Map2D = muse_smc::StateSpace<StateSpaceDescription2D>;
}

#endif // MAP_2D_HPP
